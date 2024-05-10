/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * This class provides methods for the callers to register/unregister cooperative multi-tasking tasks. It manages
 * these tasks and will work with the cooperative multi-tasking scheduler to run these tasks.
 */
public class TrcTaskMgr
{
    private static final String moduleName = TrcTaskMgr.class.getSimpleName();
    private static final TrcDbgTrace tracer = new TrcDbgTrace();

    public static long PERIODIC_INTERVAL_MS = 0;    // in msec (0: run as fast as you can)
    public static final long IO_INTERVAL_MS = 10;   // in msec
    public static final long TASKTIME_THRESHOLD_MS = PERIODIC_INTERVAL_MS > 0? PERIODIC_INTERVAL_MS * 2: 50;

    /**
     * These are the task type TrcTaskMgr supports:
     */
    public enum TaskType
    {
        /**
         * START_TASK is called on the main robot thread one time before a competition mode is about to start.
         */
        START_TASK(0),

        /**
         * STOP_TASK is called on the main robot thread one time before a competition mode is about to end.
         */
        STOP_TASK(1),

        /**
         * PRE_PERIODIC_TASK is called periodically on the main robot thread at PERIODIC_INTERVAL before runPeriodic().
         */
        PRE_PERIODIC_TASK(2),

        /**
         * POST_PERIODIC_TASK is called periodically on the main robot thread at PERIODIC_INTERVAL after runPeriodic().
         */
        POST_PERIODIC_TASK(3),

        /**
         * INPUT_TASK is called periodically on the input thread at INPUT_INTERVAL. Typically, it runs code that
         * deals with sensor hardware that may impact the performance of the main robot thread (e.g. I2C sesnors).
         */
        INPUT_TASK(4),

        /**
         * OUTPUT_TASK is called periodically on the output thread at OUTPUT_INTERVAL . Typically, it runs code that
         * deals with actuator hardware that may impact the performance of the main robot thread (e.g. CAN motors).
         */
        OUTPUT_TASK(5),

        /**
         * STANDALONE_TASK is called periodically at the specified interval on its own thread. Typically, code that
         * may block for a long time requires its own thread so that it doesn't degrade the performance of the other
         * threads.
         */
        STANDALONE_TASK(6);

        public final int value;

        TaskType(int value)
        {
            this.value = value;
        }   //TaskType

    }   //enum TaskType

    /**
     * Any class that is registering a task must implement this interface.
     */
    public interface Task
    {
        /**
         * This method is called at the appropriate time this task is registered for.
         * <p>
         * StartTask:
         *  This contains code that initializes the task before a competition mode is about to start and is run on
         *  the main robot thread. Typically, if the task is a robot subsystem, you may put last minute mode specific
         *  initialization code here. Most of the time, you don't need to register StartTask because all initialization
         *  is done in robotInit(). But sometimes, you may want to delay a certain initialization until right before
         *  competition starts. For example, you may want to reset the gyro heading right before competition starts to
         *  prevent drifting.
         * </p>
         * <p>
         * StopTask:
         *  This contains code that cleans up the task before a competition mode is about to end and is run on the main
         *  robot thread. Typically, if the task is a robot subsystem, you may put code to stop the robot here. Most of
         *  the time, you don't need to register StopTask because the system will cut power to all the motors after a
         *  competition mode has ended.
         * </p>
         * <p>
         * PrePeriodicTask:
         *  This contains code that runs periodically on the main robot thread at PERIODIC_INTERVAL before periodic().
         *  Typically, you will put code that deals with any input sensor readings here that Periodic() may depend on.
         * </p>
         * <p>
         * PostPeriodicTask:
         *  This contains code that runs periodically on the main robot thread at PERIODIC_INTERVAL after periodic().
         *  Typically, you will put code that deals with actions that requires high frequency processing.
         * </p>
         * <p>
         * InputTask:
         *  This contains code that runs periodically on the input thread. Typically, you will put code that deals
         *  with any input or sensor readings that may otherwise degrade the performance of the main robot thread.
         * </p>
         * <p>
         * OutputTask:
         *  This contains code that runs periodically on the output thread. Typically, you will put code that deals
         *  with actions that may otherwise degrade the performance of the main robot thread.
         * </p>
         * <p>
         * StandaloneTask:
         *  This contains code that will run on its own thread at the specified task interval. Typically, you will
         *  put code that may take a long time to execute and could affect the performance of shared threads such as
         *  the main robot thread, input or output threads.
         *
         * @param taskType specifies the type of task being run. This may be useful for handling multiple task types.
         * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
         * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
         *        false otherwise.
         */
        void runTask(TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop);

    }   //interface Task

    /**
     * This interface can be provided by the platform dependent task scheduler who will be called before and after each
     * IO task loop. Typically, it will contain code to initialize the hardware to start or end bulk IO cycle. If
     * platform dependent hardware doesn't require this, it doesn't need to implement it. To provide this interface,
     * call the registerIoTaskLoopBegin and registerIoTaskLoopEnd methods.
     */
    public interface IoTaskCallback
    {
        /**
         * This method is called by the IO task thread begin or after each IO task loop. It typically contains code
         * to initialize the hardware to start or end build bulk IO cycle.
         *
         * @param runMode specifies the robot run mode (e.g. Autonomous, TeleOp, Test).
         */
        void ioTaskCallback(TrcRobot.RunMode runMode);
    }   //interface IoTaskCallback

    /**
     * This class implements TaskObject that will be created whenever a class is registered as a cooperative
     * multi-tasking task. The created task objects will be entered into an array list of task objects to be
     * scheduled by the scheduler.
     */
    public static class TaskObject
    {
        private final String taskName;
        private final Task task;
        private final HashSet<TaskType> taskTypes;
        private final long[] taskStartTimes = new long[TaskType.values().length];
        private final long[] taskTotalIntervals = new long[TaskType.values().length];
        private final long[] taskTotalElapsedTimes = new long[TaskType.values().length];
        private final int[] taskTimeSlotCounts = new int[TaskType.values().length];
        private TrcPeriodicThread<Object> taskThread = null;

        /**
         * Constructor: Creates an instance of the task object with the given name
         * and the given task type.
         *
         * @param taskName specifies the instance name of the task.
         * @param task specifies the object that implements the TrcTaskMgr.Task interface.
         */
        private TaskObject(String taskName, Task task)
        {
            this.taskName = taskName;
            this.task = task;
            taskTypes = new HashSet<>();
            for (int i = 0; i < TaskType.values().length; i++)
            {
                taskStartTimes[i] = 0;
                taskTotalIntervals[i] = 0;
                taskTotalElapsedTimes[i] = 0;
                taskTimeSlotCounts[i] = 0;
            }
        }   //TaskObject

        /**
         * This method returns the instance name of the task.
         *
         * @return instance name of the class.
         */
        @Override
        public String toString()
        {
            return taskName;
        }   //toString

        /**
         * This method adds the given task type to the task object.
         *
         * @param type specifies the task type.
         * @param taskInterval specifies the periodic interval for STANDALONE_TASK, ignore for any other task types.
         *                     If zero interval is specified, the task will be run in a tight loop.
         * @param taskPriority specifies the priority of the associated thread. Only valid for STANDALONE_TASK,
         *                     ignored for any other task types.
         * @return true if successful, false if the task with that task type is already registered in the task list.
         */
        public synchronized boolean registerTask(TaskType type, long taskInterval, int taskPriority)
        {
            if (type == TaskType.STANDALONE_TASK && taskInterval < 0)
            {
                throw new IllegalArgumentException("taskInterval must be greater than or equal to 0.");
            }

            boolean added = taskTypes.add(type);

            if (added)
            {
                if (type == TaskType.STANDALONE_TASK)
                {
                    taskThread = new TrcPeriodicThread<>(taskName, this::standaloneTask, null, taskPriority);
                    taskThread.setProcessingInterval(taskInterval);
                    taskThread.setTaskEnabled(true);
                }
                else if (type == TaskType.INPUT_TASK || type == TaskType.OUTPUT_TASK)
                {
                    taskThread = null;
                    // There is only one global IO thread. All INPUT_TASKs and OUTPUT_TASKs run on this thread.
                    // The IO thread is created on first registration, so create it if not already.
                    if (ioThread == null)
                    {
                        ioThread = new TrcPeriodicThread<>(
                            moduleName + ".ioThread", TrcTaskMgr::ioTask, null, Thread.MAX_PRIORITY);
                        ioThread.setProcessingInterval(IO_INTERVAL_MS);
                        ioThread.setTaskEnabled(true);
                    }
                }
            }

            return added;
        }   //registerTask

        /**
         * This method adds the given task type to the task object.
         *
         * @param type specifies the task type.
         * @param taskInterval specifies the periodic interval for STANDALONE_TASK, ignore for any other task types.
         *                     If zero interval is specified, the task will be run in a tight loop.
         * @return true if successful, false if the task with that task type is already registered in the task list.
         */
        public boolean registerTask(TaskType type, long taskInterval)
        {
            return registerTask(type, taskInterval, Thread.NORM_PRIORITY);
        }   //registerTask

        /**
         * This method adds the given task type to the task object.
         *
         * @param type specifies the task type.
         * @return true if successful, false if the task with that task type is already registered in the task list.
         */
        public boolean registerTask(TaskType type)
        {
            return registerTask(type, 0, Thread.NORM_PRIORITY);
        }   //registerTask

        /**
         * This method removes the given task type from the task object.
         *
         * @param type specifies the task type.
         * @return true if successful, false if the task with that type is not found the task list.
         */
        public synchronized boolean unregisterTask(TaskType type)
        {
            if (type == TaskType.STANDALONE_TASK && taskThread != null)
            {
                taskThread.terminateTask();
            }
            taskThread = null;

            return taskTypes.remove(type);
        }   //unregisterTask

        /**
         * This method unregisters the given task object from all task types.
         *
         * @return true if successfully removed from any task type, false otherwise.
         */
        public synchronized boolean unregisterTask()
        {
            boolean removed = false;

            for (TaskType taskType : TaskType.values())
            {
                if (unregisterTask(taskType))
                {
                    removed = true;
                }
            }
            return removed;
        }   //unregisterTask

        /**
         * This method checks if the given task type is registered with this task object.
         *
         * @param type specifies the task type to be checked against.
         * @return true if this task is registered with the given type, false otherwise.
         */
        public synchronized boolean isRegistered(TaskType type)
        {
            return hasType(type);
        }   //isRegistered

        /**
         * This method checks if this task object is registered for any task type.
         *
         * @return true if this task is registered with any type, false otherwise.
         */
        public synchronized boolean isRegistered()
        {
            boolean registered = false;

            for (TaskType taskType : TaskType.values())
            {
                if (isRegistered(taskType))
                {
                    registered = true;
                    break;
                }
            }

            return registered;
        }   //isRegistered

        /**
         * This method checks if the given task type is registered with this task object.
         *
         * @param type specifies the task type to be checked against.
         * @return true if this task is registered as the given type, false otherwise.
         */
        private synchronized boolean hasType(TaskType type)
        {
            return taskTypes.contains(type);
        }   //hasType

        /**
         * This method returns the class object that was associated with this task object.
         *
         * @return class object associated with the task.
         */
        private Task getTask()
        {
            return task;
        }   //getTask

        /**
         * This method returns the task interval for TaskType.STANDALONE_TASK.
         *
         * @return task interval in msec. If there is no STANDALONE_TASK type in the task object, zero is returned.
         */
        public synchronized long getTaskInterval()
        {
            return taskThread != null? taskThread.getProcessingInterval(): 0;
        }   //getTaskInterval

        /**
         * This method sets the task interval for TaskType.STANDALONE_TASK. It has no effect for any other types.
         *
         * @param taskInterval specifies the periodic interval for STANDALONE_TASK, ignore for any other task types.
         *                     If zero interval is specified, the task will be run in a tight loop.
         */
        public synchronized void setTaskInterval(long taskInterval)
        {
            if (taskThread != null)
            {
                taskThread.setProcessingInterval(taskInterval);
            }
        }   //setTaskInterval

        /**
         * This method sets the task data for TaskType.STANDALONE_TASK. It has no effect for any other types.
         *
         * @param data specifies the thread data for STANDALONE_TASK, ignore for any other task types.
         */
        public synchronized void setTaskData(Object data)
        {
            if (taskThread != null)
            {
                taskThread.setData(data);
            }
        }   //setTaskData

        /**
         * This method runs the periodic standalone task.
         *
         * @param context specifies the context (not used).
         */
        private void standaloneTask(Object context)
        {
            recordStartTime(TaskType.STANDALONE_TASK);
            task.runTask(TaskType.STANDALONE_TASK, TrcRobot.getRunMode(), false);
            recordElapsedTime(TaskType.STANDALONE_TASK);
        }   //standaloneTask

        /**
         * This method records the task start timestamp in the task performance arrays. It is used to calculate
         * task elapsed time after the execution of a task.
         *
         * @param taskType specifies the task type to index into the task performance arrays.
         */
        private synchronized void recordStartTime(TaskType taskType)
        {
            long currNanoTime = TrcTimer.getNanoTime();
            long taskInterval = taskStartTimes[taskType.value] > 0 ? currNanoTime - taskStartTimes[taskType.value] : 0;

            taskStartTimes[taskType.value] = currNanoTime;
            taskTotalIntervals[taskType.value] += taskInterval;
        }   //recordStartTime

        /**
         * This method records the task elapsed time in the task performance arrays.
         *
         * @param taskType specifies the task type to index into the task performance arrays.
         */
        private synchronized void recordElapsedTime(TaskType taskType)
        {
            long currNanoTime = TrcTimer.getNanoTime();
            long startTime = taskStartTimes[taskType.value];
            long elapsedTime = currNanoTime - startTime;

            taskTotalElapsedTimes[taskType.value] += elapsedTime;
            taskTimeSlotCounts[taskType.value]++;

            if (tracer.getTraceLevel().value >= TrcDbgTrace.MsgLevel.DEBUG.value)
            {
                tracer.traceVerbose(
                    moduleName, "%s.%s: start=.6f, elapsed=%.6f",
                    taskName, taskType, (startTime/1000000000.0), elapsedTime/1000000000.0);
                long timeThreshold = getTaskInterval()*1000000; //convert to nanoseconds.
                if (timeThreshold == 0) timeThreshold = TASKTIME_THRESHOLD_MS * 1000000L;
                if (timeThreshold > 0 && elapsedTime > timeThreshold)
                {
                    tracer.traceWarn(
                        moduleName, "%s.%s takes too long (%.3f)", taskName, taskType, elapsedTime/1000000000.0);
                }
            }
        }   //recordElapsedTime

        /**
         * This method returns the average task elapsed time in seconds.
         *
         * @param taskType specifies the task type to index into the task performance arrays.
         * @return average task elapsed time in seconds.
         */
        private synchronized double getAverageTaskElapsedTime(TaskType taskType)
        {
            int slotCount = taskTimeSlotCounts[taskType.value];
            return slotCount == 0 ? 0.0 : (double)taskTotalElapsedTimes[taskType.value]/slotCount/1000000000.0;
        }   //getAverageTaskElapsedTime

        /**
         * This method returns the average task interval time in seconds.
         *
         * @param taskType specifies the task type to index into the task performance arrays.
         * @return average task interval time in seconds.
         */
        private synchronized double getAverageTaskInterval(TaskType taskType)
        {
            int slotCount = taskTimeSlotCounts[taskType.value];
            return slotCount == 0 ? 0.0 : (double)taskTotalIntervals[taskType.value]/slotCount/1000000000.0;
        }   //getAverageTaskInterval

    }   //class TaskObject

    private static final List<TaskObject> taskList = new CopyOnWriteArrayList<>();
    private static TrcPeriodicThread<Object> ioThread = null;
    private static IoTaskCallback ioTaskLoopBegin = null;
    private static IoTaskCallback ioTaskLoopEnd = null;

    /**
     * This method creates a TRC task. If the TRC task is registered as a STANDALONE task, it is run on a separately
     * created thread. Otherwise, it is run on the main robot thread as a cooperative multi-tasking task.
     *
     * @param taskName specifies the task name.
     * @param task specifies the Task interface for this task.
     * @return created task object.
     */
    public static TaskObject createTask(String taskName, Task task)
    {
        TaskObject taskObj;

        taskObj = new TaskObject(taskName, task);
        taskList.add(taskObj);
        tracer.traceDebug(moduleName, "taskName=" + taskName + ", taskObj=" + taskObj);

        return taskObj;
    }   //createTask

    /**
     * This method is mainly for FtcOpMode to call at the end of the opMode loop because runOpMode could be terminated
     * before shutdown can be called especially if stopMode code is doing logging I/O (e.g. printPerformanceMetrics).
     * This provides an earlier chance for us to stop all task threads before it's too late.
     */
    public static void terminateAllThreads()
    {
        TrcTimer.shutdown();

        for (TaskObject taskObj: taskList)
        {
            if (taskObj.hasType(TaskType.STANDALONE_TASK))
            {
                //
                // Task contains the type STANDALONE_TASK, unregister it so that the task thread will terminate.
                //
                taskObj.unregisterTask(TaskType.STANDALONE_TASK);
            }
        }

        if (ioThread != null)
        {
            ioThread.terminateTask();
            ioThread = null;
        }
    }   //terminateAllThreads

    /**
     * This method is called at the end of the robot program (FtcOpMode in FTC or FrcRobotBase in FRC) to terminate
     * all threads if any and remove all task from the task list.
     */
    public static void shutdown()
    {
        terminateAllThreads();
        taskList.clear();
    }   //shutdown

    /**
     * This method is called by the main robot thread to enumerate the task list and calls all the tasks that matches
     * the given task type.
     *
     * @param type specifies the task type to be executed.
     * @param mode specifies the robot run mode.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    public static void executeTaskType(TaskType type, TrcRobot.RunMode mode, boolean slowPeriodicLoop)
    {
        for (TaskObject taskObj: taskList)
        {
            if (taskObj.hasType(type))
            {
                Task task = taskObj.getTask();
                taskObj.recordStartTime(type);
                task.runTask(type, mode, slowPeriodicLoop);
                taskObj.recordElapsedTime(type);
            }
        }
    }   //executeTaskType

    /**
     * This method is called by the platform dependent scheduler to register callbacks at the beginning and ending
     * of the IO task loop.
     *
     * @param ioLoopBegin specifies the IO task loop begin callback, null if not provided.
     * @param ioLoopEnd specifies the IO task loop end callback, null if not provided.
     */
    public static void registerIoTaskLoopCallback(IoTaskCallback ioLoopBegin, IoTaskCallback ioLoopEnd)
    {
        ioTaskLoopBegin = ioLoopBegin;
        ioTaskLoopEnd = ioLoopEnd;
    }   //registerIoTaskLoopCallback

    /**
     * This method runs the periodic an IO task loop.
     *
     * @param context specifies the context (not used).
     */
    private static void ioTask(Object context)
    {
        TrcRobot.RunMode runMode = TrcRobot.getRunMode();

        if (ioTaskLoopBegin != null)
        {
            ioTaskLoopBegin.ioTaskCallback(runMode);
        }
        executeTaskType(TaskType.INPUT_TASK, runMode, false);
        executeTaskType(TaskType.OUTPUT_TASK, runMode, false);
        if (ioTaskLoopEnd != null)
        {
            ioTaskLoopEnd.ioTaskCallback(runMode);
        }
    }   //ioTask

    /**
     * This method prints the performance metrics of all tasks.
     */
    public static void printTaskPerformanceMetrics()
    {
        for (TaskObject taskObj: taskList)
        {
            StringBuilder msg = new StringBuilder(taskObj.taskName).append(":");
            int taskTypeCounter = 0;

            for (TaskType taskType : TaskType.values())
            {
                double taskElapsedTime = taskObj.getAverageTaskElapsedTime(taskType);
                double taskInterval = taskObj.getAverageTaskInterval(taskType);

                if (taskElapsedTime > 0.0)
                {
                    taskTypeCounter++;
                    msg.append(String.format(Locale.US, " %s=%.6f/%.6f", taskType, taskElapsedTime, taskInterval));
                }
            }

            if (taskTypeCounter > 0)
            {
                tracer.traceInfo(moduleName, msg.toString());
            }
        }
    }   //printTaskPerformanceMetrics

    /**
     * This method prints all registered tasks.
     */
    public static void printAllRegisteredTasks()
    {
        for (TaskObject taskObj : taskList)
        {
            StringBuilder msg = new StringBuilder(taskObj.toString() + ":");

            for (TaskType type : taskObj.taskTypes)
            {
                msg.append(" ");
                msg.append(type);
            }
            tracer.traceInfo(moduleName, taskObj + ": " + msg);
        }
    }   //printAllRegisteredTask

}   //class TaskMgr
