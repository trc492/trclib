/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;

import trclib.archive.TrcTaskMgr.TaskType;

/**
 * This class is a singleton. It manages a list of watchdogs. Watchdogs are typically used for detecting thread
 * deadlocks. A thread can register a watchdog with the Watchdog manager and periodically sends a heartbeat to it.
 * If a heart beat is not sent within the heart beat threshold time, a stack trace for the thread will be dump to the
 * trace log. This allows us to catch thread deadlocks quite easily. Since watchdog is designed per thread, only one
 * watchdog needs to be registered for each thread.
 */
public class TrcWatchdogMgr
{
    public static final String moduleName = TrcWatchdogMgr.class.getSimpleName();
    private static final TrcDbgTrace staticTracer = new TrcDbgTrace();

    private static final double DEF_TASK_INTERVAL = 1.0;        // in seconds.
    private static final double DEF_HEARTBEAT_THRESHOLD = 1.0;  // in seconds.

    private static TrcWatchdogMgr instance;
    private static final ArrayList<Watchdog> watchdogList = new ArrayList<>();
    private static final HashMap<Thread, Watchdog> watchdogMap = new HashMap<>();

    /**
     * This class encapsulates the state of the watchdog. A watchdog has an identifiable name, an associated thread,
     * a maximum time interval between which the heart beat must be received and the next heart beat expiraton time.
     */
    public static class Watchdog
    {
        private final String name;
        private final double heartBeatThreshold;
        private final Thread thread;
        private volatile double heartBeatExpiredTime;
        private volatile boolean expired;
        private volatile boolean paused;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the watchdog.
         * @param thread specifies the thread the watchdog is monitoring.
         * @param heartBeatThreshold specifies the maximum heart beat interval in seconds.
         * @param paused specifies true to create the watchdog in paused mode, false otherwise.
         */
        private Watchdog(String name, Thread thread, double heartBeatThreshold, boolean paused)
        {
            this.name = name;
            this.heartBeatThreshold = heartBeatThreshold;
            this.thread = thread;
            this.heartBeatExpiredTime = TrcTimer.getCurrentTime() + heartBeatThreshold;
            this.expired = false;
            this.paused = paused;
        }   //Watchdog

        /**
         * This method is called to pause watchdog monitoring. It is useful for a thread to call this before going
         * into sleep or a wait knowing it won't be able to send periodic heartbeat to prevent a watchdog timeout.
         */
        public synchronized void pauseWatch()
        {
            paused = true;
        }   //pauseWatch

        /**
         * This method is called to resume watchdog monitoring. It is useful for a thread to call this right after
         * waking up from a sleep or a wait so watchdog monitoring will be resumed.
         */
        public synchronized void resumeWatch()
        {
            paused = false;
        }   //resumeWatch

        /**
         * This method is called by the thread that registered the watchdog to send a heart beat. This will update
         * the next heart beat expiration time.
         */
        public void sendHeartBeat()
        {
            if (this.thread == Thread.currentThread())
            {
                double currTime = TrcTimer.getCurrentTime();

                synchronized (this)
                {
                    // Sending a heartbeat will also unpause a paused watchdog.
                    heartBeatExpiredTime = currTime + heartBeatThreshold;
                    expired = false;
                    paused = false;
                }
            }
            else
            {
                staticTracer.traceWarn(
                    moduleName,
                    "Only the thread created this watchdog " + name +
                    " is allowed to send heart beat (thread=" + thread.getName() + ").");
                TrcDbgTrace.printThreadStack();
            }
        }   //sendHeartBeat

        /**
         * This method checks if the watchdog has expired.
         *
         * @return true if no heartbeat has been received for the specified threshold period, false otherwise.
         */
        private synchronized boolean checkForExpiration()
        {
            double currTime = TrcTimer.getCurrentTime();

            if (!paused && !expired && currTime > heartBeatExpiredTime)
            {
                expired = true;
                staticTracer.traceWarn(moduleName, "%s expired.", this);
                TrcDbgTrace.printThreadStack(thread);
            }

            return expired;
        }   //checkForExpiration

        /**
         * This method unregisters this watchdog from Watchdog Manager.
         * Important: this method must be called in the thread the watchdog is monitoring. In other words, the
         * caller's thread must be the owner of the watchdog.
         *
         * @return true if watchdog is unregistered successfully, false if watchdog does not exist.
         */
        public boolean unregister()
        {
            boolean success = false;

            if (this.thread == Thread.currentThread())
            {
                success = unregisterWatchdog(this);
            }
            else
            {
                staticTracer.traceWarn(
                    moduleName,
                    "Only the thread created this watchdog " + name +
                    " is allowed to unregister (thread=" + thread.getName() + ").");
                TrcDbgTrace.printThreadStack();
            }

            return success;
        }   //unregsiter

        /**
         * This method returns the string containing the info of the watchdog.
         *
         * @return string form of the watchdog info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "([%.3f]%s:threshold=%f,expiredTime=%f,expired=%s,paused=s)",
                TrcTimer.getCurrentTime(), name, heartBeatThreshold, heartBeatExpiredTime, expired, paused);
        }   //toString

        /**
         * This method returns the heart beat threshold time for the watchdog.
         *
         * @return watchdog heart beat threshold time in seconds.
         */
        public double getHeartBeatThreshold()
        {
            return heartBeatThreshold;
        }   //getHeartBeatThreshold

    }   //class Watchdog

    /**
     * This method should only be called once. It creates a singleton of the Watchdog Manager. If it is called again,
     * it will just return the Watchdog Manager instance created previously.
     *
     * @param taskInterval specifies the watchdog task interval.
     * @return Watchdog Manager instance.
     */
    public static TrcWatchdogMgr getInstance(double taskInterval)
    {
        if (instance == null)
        {
            instance = new TrcWatchdogMgr(taskInterval);
        }

        return instance;
    }   //getInstance

    /**
     * This method should only be called once. It creates a singleton of the Watchdog Manager. If it is called again,
     * it will just return the Watchdog Manager instance created previously.
     *
     * @return Watchdog Manager instance.
     */
    public static TrcWatchdogMgr getInstance()
    {
        return getInstance(DEF_TASK_INTERVAL);
    }   //getInstance

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param taskInterval specifies the watchdog task interval.
     */
    private TrcWatchdogMgr(double taskInterval)
    {
        TrcTaskMgr.TaskObject watchdogTaskObj = TrcTaskMgr.createTask(moduleName, this::watchdogTask);
        watchdogTaskObj.registerTask(TaskType.STANDALONE_TASK, (long) (taskInterval*1000));
        staticTracer.traceDebug(moduleName, "Starting Watchdog Manager.");
    }   //TrcWatchdogMgr

    /**
     * This method registers a new watchdog for the current thread if one is not already registered.
     * Important: this method must be called in the thread the watchdog is monitoring.
     *
     * @param name specifies the name of the watchdog.
     * @param heartBeatThreshold specifies the maximum heart beat interval in seconds.
     * @param paused specifies true to create the watchdog in paused mode, false otherwise.
     * @return newly created watchdog.
     */
    public static Watchdog registerWatchdog(String name, double heartBeatThreshold, boolean paused)
    {
        Watchdog watchdog = null;

        instance = getInstance();
        if (staticTracer.getTraceLevel().value >= TrcDbgTrace.MsgLevel.DEBUG.value)
        {
            staticTracer.traceDebug(
                moduleName,
                "Registering watchdog " + name + " (heartBeatThreshold=" + heartBeatThreshold + ").");
            TrcDbgTrace.printThreadStack();
        }

        synchronized (watchdogList)
        {
            Thread currThread = Thread.currentThread();

            if (!watchdogMap.containsKey(currThread))
            {
                watchdog = new Watchdog(name, currThread, heartBeatThreshold, paused);
                watchdogList.add(watchdog);
                watchdogMap.put(currThread, watchdog);
            }
            else
            {
                staticTracer.traceWarn(moduleName, "Watchdog " + name + " was already registered.");
                TrcDbgTrace.printThreadStack();
            }
        }

        return watchdog;
    }   //registerWatchdog

    /**
     * This method registers a new watchdog for the current thread if one is not already registered.
     * Important: this method must be called in the thread the watchdog is monitoring.
     *
     * @param name specifies the name of the watchdog.
     * @param paused specifies true to create the watchdog in paused mode, false otherwise.
     * @return newly created watchdog.
     */
    public static Watchdog registerWatchdog(String name, boolean paused)
    {
        return registerWatchdog(name, DEF_HEARTBEAT_THRESHOLD, paused);
    }   //registerWatchdog

    /**
     * This method registers a new watchdog for the current thread if one is not already registered.
     * Important: this method must be called in the thread the watchdog is monitoring.
     *
     * @param name specifies the name of the watchdog.
     * @return newly created watchdog.
     */
    public static Watchdog registerWatchdog(String name)
    {
        return registerWatchdog(name, DEF_HEARTBEAT_THRESHOLD, false);
    }   //registerWatchdog

    /**
     * This method removes the watchdog from the watchdog list and map. It can be called from any thread as long as
     * it provides the watchdog to be unregistered.
     *
     * @param watchdog specifies the watchdog to be removed.
     * @return true if watchdog is removed successfully, false if watchdog does not exist.
     */
    public static boolean unregisterWatchdog(Watchdog watchdog)
    {
        boolean success;

        if (staticTracer.getTraceLevel().value >= TrcDbgTrace.MsgLevel.DEBUG.value)
        {
            staticTracer.traceDebug(moduleName, "Unregistering watchdog " + watchdog + ".");
            TrcDbgTrace.printThreadStack();
        }

        synchronized (watchdogList)
        {
            watchdogMap.remove(watchdog.thread);
            success = watchdogList.remove(watchdog);
        }

        if (!success)
        {
            staticTracer.traceWarn(moduleName, "Watchdog " + watchdog.name + " was never registered.");
            TrcDbgTrace.printThreadStack();
        }

        return success;
    }   //unregisterWatchdog

    /**
     * This method returns the watchdog associated with the current thread.
     *
     * @return watchdog associated with the current thread.
     */
    public static Watchdog getWatchdog()
    {
        Watchdog watchdog;

        synchronized (watchdogList)
        {
            watchdog = watchdogMap.get(Thread.currentThread());
        }

        return watchdog;
    }   //getWatchdog

    /**
     * This method runs periodically to check for watchdog timeouts.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the current robot run mode.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void watchdogTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        synchronized (watchdogList)
        {
            for (Watchdog watchdog: watchdogList)
            {
                watchdog.checkForExpiration();
            }
        }
    }   //watchdogTask

}   //class TrcWatchdogMgr
