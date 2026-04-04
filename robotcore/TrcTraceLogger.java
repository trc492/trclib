/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.concurrent.LinkedBlockingQueue;

import trclib.timer.TrcTimer;

public class TrcTraceLogger
{
    public final TrcDbgTrace tracer;
    private final String traceLogName;
    private final LinkedBlockingQueue<String> msgQueue;

    private PrintWriter traceLog = null;
    private volatile Thread loggerThread = null;
    private volatile boolean enabled = false;
    private String newLogName = null;
    private double totalNanoTime = 0.0;
    private int totalMessages = 0;

    /**
     * Constructor: Create an instance of the trace logger.
     *
     * @param traceLogName specifies the log file name.
     */
    public TrcTraceLogger(String traceLogName)
    {
        this.tracer = new TrcDbgTrace();
        this.traceLogName = traceLogName;
        msgQueue = new LinkedBlockingQueue<>();
        // Open the log file for append and create the logger thread.
        try
        {
            traceLog = new PrintWriter(new BufferedWriter(new FileWriter(traceLogName, true)));
        }
        catch (IOException e)
        {
            e.printStackTrace();
            throw new RuntimeException("Failed to open trace log file " + traceLogName);
        }
        loggerThread = new Thread(this::loggerTask, traceLogName);
        loggerThread.start();
    }   //TrcTraceLogger

    /**
     * This method returns the trace log name.
     *
     * @return trace log name, "None" if no log file opened.
     */
    @Override
    public String toString()
    {
        return traceLogName;
    }   //toString

    /**
     * This method closes the trace logger and renames the log file.
     *
     * @param newLogName specifies the new log file name. If provided, the log file will be renamed to the new name.
     */
    public synchronized void closeLogger(String newLogName)
    {
        // If the new log name is the same as the current log name, we don't need to rename it.
        this.newLogName = !traceLogName.equals(newLogName)? newLogName: null;
        this.enabled = false;
        loggerThread.interrupt();
    }   //closeLogger

    /**
     * This method enables/disables the trace logger thread.
     *
     * @param enabled specifies true to enable logger thread, false to disable.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        this.enabled = enabled;
    }   //setEnabled

    /**
     * This method checks if the trace log is enabled.
     *
     * @return true if trace log is enabled, false if disabled.
     */
    public synchronized boolean isEnabled()
    {
        return enabled;
    }   //isEnabled

    /**
     * This method is called to log a message to the log file.
     *
     * @param msg specifies the message to be logged.
     */
    public synchronized boolean logMessage(String msg)
    {
        boolean success = false;

        if (enabled)
        {
            success = msgQueue.add(msg);
        }

        return success;
    }   //logMessage

    /**
     * This method writes the message to the trace log and also keeps track of logging performance.
     *
     * @param msg specifies the message to be logged.
     */
    private void writeMessage(String msg)
    {
        long startNanoTime = TrcTimer.getNanoTime();
        traceLog.print(msg + "\r\n");
        traceLog.flush();
        double elapsedNanoTime = TrcTimer.getNanoTime() - startNanoTime;
        totalNanoTime += elapsedNanoTime;
        totalMessages++;
        //
        // Make sure we don't recursively log the performance message itself.
        //
        if (!msg.startsWith(traceLogName))
        {
            tracer.traceDebug(
                traceLogName, "Avg message log time=" +  (totalNanoTime/totalMessages/1000000000.0 + " sec"));
        }
    }   //writeMessage

    /**
     * This method closes the trace log file and renames it if a new log name is provided.
     * It is called when the logger thread is terminating.
     */
    private void closeLogFile()
    {
        if (traceLog != null)
        {
            traceLog.close();
            traceLog = null;
            if (newLogName != null)
            {
                Path source = Paths.get(traceLogName);
                Path target = Paths.get(newLogName);

                try
                {
                    Files.move(source, target, StandardCopyOption.REPLACE_EXISTING);
                }
                catch (IOException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }   //closeLogFile

    /**
     * This method is called when the logger thread is started. It processes all messages in the message queue when
     * they arrive. If the message queue is empty, the thread is blocked until a new message arrives. Therefore,
     * this thread only runs when there are messages in the queue. If this thread is interrupted, it will exit
     * only after all the remaining messages in the queue are written to the log.
     */
    private void loggerTask()
    {
        String msg;

        tracer.traceDebug(traceLogName, "Trace Logger starting...");
        while (!Thread.currentThread().isInterrupted())
        {
            try
            {
                msg = msgQueue.take();
                writeMessage(msg);
                tracer.traceDebug(traceLogName, "Logging message <" + msg + ">");
            }
            catch (InterruptedException e)
            {
                tracer.traceDebug(traceLogName, "Terminating Trace Logger");
                break;
            }
        }
        //
        // The thread is terminating, empty the queue before exiting.
        //
        while ((msg = msgQueue.poll()) != null)
        {
            writeMessage(msg);
            tracer.traceDebug(traceLogName, "Emptying message <" + msg + ">");
        }
        tracer.traceDebug(traceLogName, "Closing Trace Log");

        closeLogFile();
        loggerThread = null;
    }   //loggerTask

}   //class TrcTraceLogger
