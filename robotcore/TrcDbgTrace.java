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

package trclib.robotcore;

import java.io.File;
import java.util.Locale;

import trclib.path.TrcPidDrive;
import trclib.path.TrcPose2D;
import trclib.path.TrcPurePursuitDrive;
import trclib.sensor.TrcRobotBattery;
import trclib.drivebase.TrcDriveBase;
import trclib.timer.TrcTimer;

/**
 * This class implements the Debug Tracer.
 */
public class TrcDbgTrace
{
    private static final String moduleName = TrcDbgTrace.class.getSimpleName();

    /**
     * This enum specifies the different debug message levels. They are used in the traceMsg methods.
     */
    public enum MsgLevel
    {
        FATAL(1),
        ERR(2),
        WARN(3),
        INFO(4),
        DEBUG(5),
        VERBOSE(6);

        final int value;

        MsgLevel(int value)
        {
            this.value = value;
        }   //MsgLevel

    }   //enum MsgLevel

    /**
     * This interface provides a platform independent way to write to the debug log. It is mainly for TrcLib which is
     * platform agnostic. A platform dependent class will implement methods in this interface.
     */
    public interface DbgLog
    {
        /**
         * This method is called to print a message with the specified message level to the debug console.
         *
         * @param level specifies the message level.
         * @param msg specifies the message.
         */
        void msg(MsgLevel level, String msg);

    }   //interface DbgLog

    private static DbgLog dbgLog = null;
    private static TrcDbgTrace globalTracer = null;
    private static TrcTraceLogger traceLogger = null;

    private MsgLevel msgLevel = MsgLevel.INFO;

    /**
     * Constructor: Create an instance of the object. This constructor is intended for internal use for creating
     * the global tracer. For creating other tracers, use the constructor with just instanceName.
     *
     * @param callerName specifies the module name of the caller.
     * @param dbgLog specifies the dbgLog object to be set.
     */
    public TrcDbgTrace(String callerName, DbgLog dbgLog)
    {
        if (callerName != null && dbgLog != null)
        {
            if (TrcDbgTrace.dbgLog == null && callerName.equals("FrcRobotBase") || callerName.equals("FtcOpMode"))
            {
                TrcDbgTrace.dbgLog = dbgLog;
                TrcDbgTrace.globalTracer = this;
            }
            else
            {
                throw new IllegalStateException(
                    "This constructor can only be called internally to create the global Tracer.");
            }
        }
        else if (callerName != null ^ dbgLog != null)
        {
            throw new IllegalStateException(
                "This constructor can only be called internally to create the global Tracer.");
        }
    }   //TrcDbgTrace

    /**
     * Constructor: Create an instance of the object.
     */
    public TrcDbgTrace()
    {
        this(null, null);
    }   //TrcDbgTrace

    /**
     * This method sets the message level for this tracer.
     *
     * @param msgLevel specifies the message level.
     */
    public void setTraceLevel(MsgLevel msgLevel)
    {
        this.msgLevel = msgLevel;
    }   //setTraceLevel

    /**
     * This method returns the trace message level.
     *
     * @return trace message level.
     */
    public MsgLevel getTraceLevel()
    {
        return msgLevel;
    }   //getTraceLevel

    /**
     * This method returns a global debug trace object for tracing OpMode code. If it doesn't exist yet, one is
     * created. This is an easy way to quickly get some debug output without a whole lot of setup overhead as the
     * full module-based debug tracing.
     *
     * @return global opMode trace object.
     */
    public static TrcDbgTrace getGlobalTracer()
    {
        return globalTracer;
    }   //getGlobalTracer

    /**
     * This method opens a log file for writing all the trace messages to it.
     *
     * @param traceLogName specifies the full trace log file path name.
     * @return true if log file is successfully opened, false if it failed.
     */
    public static boolean openTraceLog(String traceLogName)
    {
        boolean success = false;

        if (traceLogger == null)
        {
            traceLogger = new TrcTraceLogger(traceLogName);
            success = true;
        }

        return success;
    }   //openTraceLog

    /**
     * This method opens a log file for writing all the trace messages to it. The log file is written to the specified
     * folder. The file name will be formed by concatenating the date-time stamp with the specified file name.
     *
     * @param folderPath specifies the folder path.
     * @param fileName specifies the file name, null if none provided.
     * @return true if log file is successfully opened, false if it failed.
     */
    public static boolean openTraceLog(String folderPath, String fileName)
    {
        //
        // Create the folder if it doesn't exist.
        //
        File folder = new File(folderPath);
        if (!folder.exists())
        {
            folder.mkdirs();
        }
        //
        // Create full log file path.
        //
        String logFileName = folderPath + File.separator + TrcTimer.getCurrentTimeString();

        if (fileName != null)
        {
            logFileName += "!" + fileName;
        }
        logFileName += ".log";

        return openTraceLog(logFileName);
    }   //openTraceLog

    /**
     * This method closes the trace log file.
     */
    public static void closeTraceLog()
    {
        if (traceLogger != null)
        {
            traceLogger.setEnabled(false);
            traceLogger = null;
        }
    }   //closeTraceLog

    /**
     * This method checks if the trace log is opened.
     *
     * @return true if trace log is opened, false otherwise.
     */
    public static boolean isTraceLogOpened()
    {
        return traceLogger != null;
    }   //isTraceLogOpened

    /**
     * This method returns the trace log file name if one is active.
     *
     * @return trace log file name if one is active, null if none.
     */
    public static String getTraceLogName()
    {
        return traceLogger != null? traceLogger.toString(): null;
    }   //getTraceLogName

    /**
     * This method enables/disables the trace log.
     *
     * @param enabled specifies true to enable trace log, false otherwise.
     */
    public static void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogger != null)
        {
            traceLogger.setEnabled(enabled);
        }
    }   //setTraceLogEnabled

    /**
     * This method checks if the trace log is enabled.
     *
     * @return true if trace log is enabled, false if disabled.
     */
    public static boolean isTraceLogEnabled()
    {
        return (traceLogger != null && traceLogger.isEnabled());
    }   //isTraceLogEnabled

    /**
     * This method prints the exception stack to the global tracer.
     *
     * @param e specifies the exception.
     */
    public static void printExceptionStack(Exception e)
    {
        StackTraceElement[] stackTraceElements = e.getStackTrace();
        StringBuilder sb = new StringBuilder("Stack Trace (depth=").append(stackTraceElements.length).append("):");

        for (StackTraceElement ste : stackTraceElements)
        {
            sb.append("\n").append(ste);
        }
        globalTracer.traceMsgWorker(moduleName, 2, MsgLevel.ERR, sb.toString());
    }   //printExceptionStack

    /**
     * This method prints the stack of the given thread to the trace log.
     *
     * @param thread specifies the thread to print its stack.
     */
    public static void printThreadStack(Thread thread)
    {
        StringBuilder sb = new StringBuilder("Thread stack: ");

        for (StackTraceElement ste : thread.getStackTrace())
        {
            sb.append("\n").append(ste);
        }

        globalTracer.traceMsgWorker(moduleName, 2, MsgLevel.INFO, sb.toString());
    }   //printThreadStack

    /**
     * This method prints the stack of the current thread to the trace log.
     */
    public static void printThreadStack()
    {
        printThreadStack(Thread.currentThread());
    }   //printThreadStack

    /**
     * This method checks if the specified msgLevel is enabled.
     *
     * @param level specifies the msgLevel.
     * @return true if enabled, false otherwise.
     */
    public boolean isMsgLevelEnabled(MsgLevel level)
    {
        return level.value <= msgLevel.value;
    }   //isMsgLevelEnabled

    /**
     * This method is the common worker for all the trace message methods.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param methodIndex specifies the index to the thread stack to obtain the method name.
     * @param level specifies the message level.
     * @param text specifies the message text.
     */
    private void traceMsgWorker(String callerInstance, int methodIndex, MsgLevel level, String text)
    {
        if (level.value <= msgLevel.value)
        {
            String msg =
                callerInstance + "." + new Throwable().getStackTrace()[methodIndex].getMethodName() + "_" + level +
                " [" + TrcTimer.getModeElapsedTime() + "] " + text;
            dbgLog.msg(level, msg + "\n");
            if (traceLogger != null)
            {
                traceLogger.logMessage(msg);
            }
        }
    }   //traceMsgWorker

    /**
     * This method is called to print a fatal message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceFatal(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.FATAL.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.FATAL, text);
        }
    }   //traceFatal

    /**
     * This method is called to print a fatal message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceFatal(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.FATAL.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.FATAL, String.format(format, args));
        }
    }   //traceFatal

    /**
     * This method is called to print an error message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceErr(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.ERR.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.ERR, text);
        }
    }   //traceErr

    /**
     * This method is called to print an error message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceErr(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.ERR.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.ERR, String.format(format, args));
        }
    }   //traceErr

    /**
     * This method is called to print a warning message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceWarn(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.WARN.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.WARN, text);
        }
    }   //traceWarn

    /**
     * This method is called to print a warning message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceWarn(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.WARN.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.WARN, String.format(format, args));
        }
    }   //traceWarn

    /**
     * This method is called to print an information message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceInfo(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.INFO, text);
        }
    }   //traceInfo

    /**
     * This method is called to print an information message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceInfo(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.INFO, String.format(format, args));
        }
    }   //traceInfo

    /**
     * This method is called to print a debug message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceDebug(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.DEBUG.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.DEBUG, text);
        }
    }   //traceDebug

    /**
     * This method is called to print a debug message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceDebug(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.DEBUG.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.DEBUG, String.format(format, args));
        }
    }   //traceDebug

    /**
     * This method is called to print a verbose message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceVerbose(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.VERBOSE.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.VERBOSE, text);
        }
    }   //traceVerbose

    /**
     * This method is called to print a verbose message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceVerbose(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.VERBOSE.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.VERBOSE, String.format(format, args));
        }
    }   //traceVerbose

    /**
     * This method is called to print a fatal message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceFatal(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.FATAL.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.FATAL, text);
        }
    }   //globalTraceFatal

    /**
     * This method is called to print a fatal message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceFatal(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.FATAL.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.FATAL, String.format(format, args));
        }
    }   //globalTraceFatal

    /**
     * This method is called to print an error message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceErr(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.ERR.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.ERR, text);
        }
    }   //globalTraceErr

    /**
     * This method is called to print an error message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceErr(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.ERR.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.ERR, String.format(format, args));
        }
    }   //globalTraceErr

    /**
     * This method is called to print a warning message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceWarn(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.WARN.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.WARN, text);
        }
    }   //globalTraceWarn

    /**
     * This method is called to print a warning message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceWarn(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.WARN.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.WARN, String.format(format, args));
        }
    }   //globalTraceWarn

    /**
     * This method is called to print an information message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceInfo(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.INFO.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.INFO, text);
        }
    }   //globalTraceInfo

    /**
     * This method is called to print an information message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceInfo(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.INFO.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.INFO, String.format(format, args));
        }
    }   //globalTraceInfo

    /**
     * This method is called to print a debug message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceDebug(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.DEBUG.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.DEBUG, text);
        }
    }   //globalTraceDebug

    /**
     * This method is called to print a debug message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceDebug(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.DEBUG.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.DEBUG, String.format(format, args));
        }
    }   //globalTraceDebug

    /**
     * This method is called to print a verbose message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceVerbose(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.VERBOSE.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.VERBOSE, text);
        }
    }   //globalTraceVerbose

    /**
     * This method is called to print a verbose message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceVerbose(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.VERBOSE.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.VERBOSE, String.format(format, args));
        }
    }   //globalTraceVerbose

    /**
     * This method logs a MsgLevel.INFO entry that contains information about the match. The entry is in XML format
     * and is intended to be parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param infoName specifies the name to identify the information.
     * @param text specifies the message text.
     */
    public void logInfo(String callerInstance, String infoName, String text)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.INFO, "<Info name=\"" + infoName + "\" " + text + " />");
        }
    }   //logInfo

    /**
     * This method logs a MsgLevel.INFO entry that contains information about the match. The entry is in XML format
     * and is intended to be parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param infoName specifies the name to identify the information.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void logInfo(String callerInstance, String infoName, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(
                callerInstance, 2, MsgLevel.INFO,
                "<Info name=\"" + infoName + "\" " + String.format(format, args) + " />");
        }
    }   //logInfo

    /**
     * This method logs a MsgLevel.INFO entry that contains an event. The entry is in XML format and is intended to be
     * parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param eventName specifies the name to identify the event.
     * @param text specifies the message text.
     */
    public void logEvent(String callerInstance, String eventName, String text)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(
                callerInstance, 2, MsgLevel.INFO,
                "<Event name=\"" + eventName + "\" time=\"" + TrcTimer.getModeElapsedTime() + "\" " + text + " />");
        }
    }   //logEvent

    /**
     * This method logs a MsgLevel.INFO entry that contains an event. The entry is in XML format and is intended to be
     * parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param eventName specifies the name to identify the event.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void logEvent(String callerInstance, final String eventName, final String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(
                callerInstance, 2, MsgLevel.INFO,
                "<Event name=\"" + eventName + "\" time=\"" + TrcTimer.getModeElapsedTime() + "\" " +
                String.format(format, args) + " />");
        }
    }   //logEvent

    /**
     * This method logs a MsgLevel.INFO entry that contains an event. The entry is in XML format and is intended to be
     * parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param methodIndex specifies the index to the thread stack to obtain the method name.
     * @param eventName specifies the name to identify the event.
     * @param text specifies the message text.
     */
    private void logEventInternal(String callerInstance, int methodIndex, String eventName, String text)
    {
        traceMsgWorker(
            callerInstance, methodIndex, MsgLevel.INFO,
            "<Event name=\"" + eventName + "\" time=\"" + TrcTimer.getModeElapsedTime() + "\" " + text + " />");
    }   //logEventInternal

    /**
     * This method logs a pre-state info event using the global tracer. The pre-state info event can be used to
     * debug a state machine when the state is about to be executed.
     *
     * @param name specifies the instance name of the state machine.
     * @param state specifies the current state of the state machine.
     */
    public void tracePreStateInfo(String name, Object state)
    {
        if (msgLevel.value >= MsgLevel.INFO.value && state != null)
        {
            logEventInternal(name, 3, "StateInfo", "tag=\"vvvvv\" " + name + ".state=\"" + state + "\"");
        }
    }   //tracePreStateInfo

    /**
     * This method logs a post-state info event using the global tracer. The post-state info event can be used to
     * debug a state machine after the state was executed. If the state involves PID controlled driving, it also
     * logs the robot's movement. Post-state info contains information of the async operations it has started but not
     * necessarily finished.
     *
     * @param name specifies the instance name of the state machine.
     * @param state specifies the current state of the state machine.
     * @param driveBase specifies the robot drive base, can be null if the state does not involve robot movement.
     * @param pidDrive specifies the pidDrive object, can be null if the state does not involve robot movement.
     * @param ppDrive specifies the purePursuitDrive object, can be null if the state does not involve pp drive.
     * @param battery specifies the robot battery object, can be null if not interested in battery info.
     */
    private void tracePostStateInfoWorker(
        String name, Object state, TrcDriveBase driveBase, TrcPidDrive pidDrive, TrcPurePursuitDrive ppDrive,
        TrcRobotBattery battery)
    {
        if (state != null)
        {
            StringBuilder msg = new StringBuilder("tag=\"^^^^^\" " + name + ".state=\"" + state + "\"");

            if (driveBase != null)
            {
                if (pidDrive != null && pidDrive.isActive())
                {
                    TrcPose2D robotPose = driveBase.getFieldPosition();
                    TrcPose2D targetPose = pidDrive.getAbsoluteTargetPose();
                    msg.append(" RobotPose=")
                       .append(robotPose)
                       .append(" TargetPose=")
                       .append(targetPose);
                }

                if (ppDrive != null && ppDrive.isActive())
                {
                    TrcPose2D robotPose = driveBase.getFieldPosition();
                    TrcPose2D robotVel = driveBase.getFieldVelocity();
                    TrcPose2D targetPose = ppDrive.getTargetFieldPosition();
                    msg.append(" RobotPose=")
                       .append(robotPose)
                       .append(" TargetPose=")
                       .append(targetPose)
                       .append(" vel=")
                       .append(robotVel)
                       .append(" Path=")
                       .append(ppDrive.getPath());
                }
            }

            if (battery != null)
            {
                msg.append(String.format(
                    Locale.US, " volt=\"%.3fV(%.3fV)\"", battery.getVoltage(), battery.getLowestVoltage()));
            }

            logEventInternal(name, 4, "StateInfo", msg.toString());
        }
    }   //tracePostStateInfoWorker

    /**
     * This method logs a post-state info event using the global tracer. The post-state info event can be used to
     * debug a state machine after the state was executed. If the state involves PID controlled driving, it also
     * logs the robot's movement. Post-state info contains information of the async operations it has started but not
     * necessarily finished.
     *
     * @param name specifies the instance name of the state machine.
     * @param state specifies the current state of the state machine.
     * @param driveBase specifies the robot drive base, can be null if the state does not involve robot movement.
     * @param pidDrive specifies the pidDrive object, can be null if the state does not involve robot movement.
     * @param ppDrive specifies the purePursuitDrive object, can be null if the state does not involve pp drive.
     * @param battery specifies the robot battery object, can be null if not interested in battery info.
     */
    public void tracePostStateInfo(
        String name, Object state, TrcDriveBase driveBase, TrcPidDrive pidDrive, TrcPurePursuitDrive ppDrive,
        TrcRobotBattery battery)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            tracePostStateInfoWorker(name, state, driveBase, pidDrive, ppDrive, battery);
        }
    }   //tracePostStateInfo

    /**
     * This method logs a post-state info event using the global tracer. The post-state info event can be used to
     * debug a state machine after the state was executed. If the state involves PID controlled driving, it also
     * logs the robot's movement. Post-state info contains information of the async operations it has started but not
     * necessarily finished.
     *
     * @param name specifies the instance name of the state machine.
     * @param state specifies the current state of the state machine.
     * @param driveBase specifies the robot drive base, can be null if the state does not involve robot movement.
     * @param pidDrive specifies the pidDrive object, can be null if the state does not involve robot movement.
     */
    public void tracePostStateInfo(String name, Object state, TrcDriveBase driveBase, TrcPidDrive pidDrive)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            tracePostStateInfoWorker(name, state, driveBase, pidDrive, null, null);
        }
    }   //tracePostStateInfo

    /**
     * This method logs a post-state info event using the global tracer. The post-state info event can be used to
     * debug a state machine after the state was executed. If the state involves PID controlled driving, it also
     * logs the robot's movement. Post-state info contains information of the async operations it has started but not
     * necessarily finished.
     *
     * @param name specifies the instance name of the state machine.
     * @param state specifies the current state of the state machine.
     * @param driveBase specifies the robot drive base, can be null if the state does not involve robot movement.
     * @param ppDrive specifies the purePursuitDrive object, can be null if the state does not involve pp drive.
     * @param battery specifies the robot battery object, can be null if not interested in battery info.
     */
    public void tracePostStateInfo(String name, Object state, TrcDriveBase driveBase, TrcPurePursuitDrive ppDrive)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            tracePostStateInfoWorker(name, state, driveBase, null, ppDrive, null);
        }
    }   //tracePostStateInfo

    /**
     * This method logs a post-state info event using the global tracer. The post-state info event can be used to
     * debug a state machine after the state was executed. If the state involves PID controlled driving, it also
     * logs the robot's movement. Post-state info contains information of the async operations it has started but not
     * necessarily finished.
     *
     * @param name specifies the instance name of the state machine.
     * @param state specifies the current state of the state machine.
     * @param battery specifies the robot battery object, can be null if not interested in battery info.
     */
    public void tracePostStateInfo(String name, Object state, TrcRobotBattery battery)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            tracePostStateInfoWorker(name, state, null, null, null, battery);
        }
    }   //tracePostStateInfo

}   //class TrcDbgTrace
