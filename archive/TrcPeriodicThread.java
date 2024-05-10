/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib.archive;

import java.util.concurrent.atomic.AtomicInteger;

/**
 * This class implements a platform independent periodic task by using a separate thread. When enabled, the thread
 * periodically calls the runPeriodic method. Generally, this class is used by TrcTaskMgr to create a standalone
 * thread that runs the periodic task when STANDALONE_TASK is registered. By doing so, the standalone task doesn't
 * slow down the main robot thread even if it is performing something that may block for long period of time.
 * In rare occasions, this class can also be used by a sensor that requires long period of time to acquire its
 * data but arguably, one could just call TrcTaskMgr to create a STANDALONE_TASK instead. In other words, this
 * class is mainly used by TrcTaskMgr, there is really no reason for others to use this class. One should always
 * use TrcTaskMgr to create a STANDALONE_TASK.
 *
 * @param <T> specifies the data type that the periodic task will be acquiring/processing.
 */
public class TrcPeriodicThread<T>
{
    private static boolean robotInitialized = false;

    public interface PeriodicTask
    {
        void runPeriodic(Object context);
    }   //interface PeriodicTask

    /**
     * This class keeps track of the state of the periodic task. It also provides thread synchronization control to
     * make sure the integrity of the task state.
     */
    private class TaskState
    {
        private final Thread periodicThread;
        private boolean taskEnabled;
        private T data;

        /**
         * Constructor: Create an instance of the object.
         */
        public TaskState(String instanceName, Runnable runnable, int taskPriority)
        {
            periodicThread = new Thread(runnable, instanceName);
            periodicThread.setPriority(taskPriority);
            taskEnabled = false;
            data = null;
        }   //TaskState

        /**
         * This method is called after TaskState is created to start the task thread.
         */
        public void start()
        {
            periodicThread.start();
        }   //start

        /**
         * This method checks if the periodic task has been terminated.
         *
         * @return true if task has been terminated, false otherwise.
         */
        public boolean isTaskTerminated()
        {
            return !periodicThread.isAlive();
        }   //isTaskTerminated

        /**
         * This method is called to terminate the periodic task.
         */
        public void terminateTask()
        {
            periodicThread.interrupt();
        }   //terminateTask

        /**
         * This method checks if the periodic task is enabled.
         *
         * @return true if task is enabled, false otherwise.
         */
        public synchronized boolean isTaskEnabled()
        {
            return robotInitialized && taskEnabled && periodicThread.isAlive();
        }   //isTaskEnabled

        /**
         * This method enables/disables the periodic task. If this is called to disable the task, the task will be
         * set to a paused state. The operation will be resumed when this is called to enable it again.
         *
         * @param enabled specifies true to enable periodic task, false to disable.
         */
        public synchronized void setTaskEnabled(boolean enabled)
        {
            if (periodicThread.isAlive())
            {
                taskEnabled = enabled;
            }
            else
            {
                //
                // Thread is not alive, make sure the task state is disabled.
                //
                taskEnabled = false;
            }
        }   //setTaskEnabled

        /**
         * This method returns the last data object associated with the task. If there is no new data since the last
         * call, it will return null. This method, along with setData(), provides a thread-safe way to access the data
         * produced or processed by the thread.
         *
         * @return newly acquired data if any, null if none.
         */
        public synchronized T getData()
        {
            T newData = null;

            if (periodicThread.isAlive())
            {
                //
                // Consume the data by transferring it out.
                //
                newData = data;
                data = null;
            }

            return newData;
        }   //getData

        /**
         * This method is called to set new data after new data have been acquired/processed. This method, along with
         * getData(), provides a thread-safe way to access the data produced or processed by the thread.
         *
         * @param data specifies newly acquired/processed data. 
         */
        public synchronized void setData(T data)
        {
            if (periodicThread.isAlive())
            {
                this.data = data;
            }
        }   //setData

    }   //class TaskState

    private static final AtomicInteger numActiveThreads = new AtomicInteger(0);
    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final PeriodicTask task;
    private final Object context;
    private final TaskState taskState;
    private volatile long processingInterval = 0;   // in msec

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param task specifies the periodic task the thread is to execute.
     * @param context specifies the task context to be passed to the periodic thread.
     * @param taskPriority specifies the periodic thread priority.
     */
    public TrcPeriodicThread(String instanceName, PeriodicTask task, Object context, int taskPriority)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.task = task;
        this.context = context;
        taskState = new TaskState(instanceName, this::run, taskPriority);
        taskState.start();
    }   //TrcPeriodicThread

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param task specifies the periodic task the thread is to execute.
     * @param context specifies the task context to be passed to the periodic thread.
     */
    public TrcPeriodicThread(String instanceName, PeriodicTask task, Object context)
    {
        this(instanceName, task, context, Thread.NORM_PRIORITY);
    }   //TrcPeriodicThread

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method is called by the framework scheduler to inform us whether robotInit has finished. Periodic threads
     * won't be executing until robotInit has finished.
     *
     * @param init specifies true if robotInit has finished, false otherwise.
     */
    public static void setRobotInitialized(boolean init)
    {
        robotInitialized = init;
    }   //setRobotInitialized

    /**
     * This method returns the number of active threads.
     *
     * @return number of active threads.
     */
    public static int getNumActiveThreads()
    {
        return numActiveThreads.get();
    }   //getNumActiveThreads

    /**
     * This method is called to terminate the periodic task. Once this is called, no other method in this class
     * should be called except for isTaskTerminated().
     */
    public void terminateTask()
    {
        taskState.terminateTask();
    }   //terminateTask

    /**
     * This method checks if the periodic task has been terminated.
     *
     * @return true if periodic task is terminated, false otherwise.
     */
    public boolean isTaskTerminated()
    {
        return taskState.isTaskTerminated();
    }   //isTaskTerminated

    /**
     * This method enables/disables the periodic task. As long as the task is enabled, it will continue to
     * acquire/process data.
     *
     * @param enabled specifies true to enable periodic task, false to disable.
     */
    public void setTaskEnabled(boolean enabled)
    {
        taskState.setTaskEnabled(enabled);
    }   //setTaskEnabled

    /**
     * This method returns the state of the periodic task.
     *
     * @return true if the periodic task is enabled, false otherwise.
     */
    public boolean isTaskEnabled()
    {
        return taskState.isTaskEnabled();
    }   //isTaskEnabled

    /**
     * This method sets the periodic task processing interval.
     *
     * @param interval specifies the processing interval in msec. If 0, process as fast as the CPU can run.
     */
    public void setProcessingInterval(long interval)
    {
        processingInterval = interval;
    }   //setProcessInterval

    /**
     * This method returns the periodic task processing interval.
     *
     * @return periodic task processing interval in msec. If zero, the task is run as fast as the CPU can go.
     */
    public long getProcessingInterval()
    {
        return processingInterval;
    }   //getProcessingInterval

    /**
     * This method is called to set new data after new data have been acquired/processed.
     *
     * @param data specifies newly acquired/processed data. 
     */
    public void setData(T data)
    {
        taskState.setData(data);
    }   //setData

    /**
     * This method returns the acquired/processed data. If nothing found, it returns null.
     *
     * @return acquired/processed data, null if nothing found.
     */
    public T getData()
    {
        return taskState.getData();
    }   //getData

    //
    // Implements Runnable interface.
    //

    /**
     * This method runs the periodic processing task.
     */
    public void run()
    {
        final Thread thread = Thread.currentThread();
        @SuppressWarnings("unused") long totalThreadNanoTime = 0;
        @SuppressWarnings("unused") int loopCount = 0;

        int numThreads = numActiveThreads.incrementAndGet();

        tracer.traceDebug(
            instanceName,
            "Thread " + numThreads +
            ": tid=" + thread.getId() +
            ", priority=" + thread.getPriority() +
            ", group=" + thread.getThreadGroup());
        // Do not create a watchdog for the Watchdog Manager task.
        TrcWatchdogMgr.Watchdog threadWatchdog =
            instanceName.equals(TrcWatchdogMgr.moduleName)? null: TrcWatchdogMgr.registerWatchdog(instanceName);
        TrcEvent.registerEventCallback();
        while (!Thread.interrupted())
        {
            long startNanoTime = TrcTimer.getNanoTime();
            long elapsedNanoTime;

            if (taskState.isTaskEnabled())
            {
                task.runPeriodic(context);
                elapsedNanoTime = TrcTimer.getNanoTime() - startNanoTime;
                totalThreadNanoTime += elapsedNanoTime;
                loopCount++;
                tracer.traceVerbose(
                    instanceName, "start=%.6f, elapsed=%.6f", startNanoTime/1000000000.0, elapsedNanoTime/1000000000.0);
            }

            TrcEvent.performEventCallback();
            if (threadWatchdog != null)
            {
                threadWatchdog.sendHeartBeat();
            }

            if (processingInterval > 0)
            {
                long sleepTime = processingInterval - (TrcTimer.getNanoTime() - startNanoTime)/1000000;
                //
                // If the processing time does not use up the processingInterval time, make the thread sleep the
                // remaining time left.
                //
                if (sleepTime > 0)
                {
                    try
                    {
                        Thread.sleep(sleepTime);
                    }
                    catch (InterruptedException e)
                    {
                        break;
                    }
                }
            }
            else
            {
                //
                // If the thread does not have a processing interval, we should at least yield to prevent going into
                // a tight loop possibly doing nothing.
                //
                Thread.yield();
            }
        }

        TrcEvent.unregisterEventCallback();
        if (threadWatchdog != null)
        {
            threadWatchdog.unregister();
        }

        numThreads = numActiveThreads.decrementAndGet();
        tracer.traceDebug(
            instanceName, "Exiting thread: numThreads=%d, AvgLoopTime=%.6f",
            numThreads, totalThreadNanoTime/1000000000.0/loopCount);
    }   //run

}   //class TrcPeriodicThread
