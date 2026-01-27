/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.HashMap;
import java.util.Locale;

import trclib.timer.TrcTimer;

/**
 * This class implements a loop profiler for recording the elapsed time of various segments of code in a periodic
 * loop. This is useful to identify the culprit if the loop time is unacceptably long. This can be used in profiling
 * task loops, periodic thread loops or even the main robot loop.
 */
public class TrcLoopProfiler
{
    private static class ProfilePoint
    {
        String name;
        boolean inLoop;
        int index;
        long totalElapsedTime;

        public ProfilePoint(String name, boolean inLoop, int index)
        {
            this.name = name;
            this.inLoop = inLoop;
            this.index = index;
            totalElapsedTime = 0;
        }   //ProfilePoint

        @Override
        public String toString()
        {
            return "(name=" + name + ",inLoop=" + inLoop + ",index=" + index +
                   ",totalElapsedTime=" + totalElapsedTime + ")";
        }   //toString

    }   //class ProfilePoint

    private final HashMap<String, ProfilePoint> profilePoints = new HashMap<>();
    private long loopStartNanoTime = 0;
    private long minLoopIntervalTime = Long.MAX_VALUE;
    private Long maxLoopIntervalTime = Long.MIN_VALUE;
    private long totalLoopIntervalTime = 0;
    private long loopCount = 0;

    private final String instanceName;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the name to identify this instance of the loop profiler.
     */
    public TrcLoopProfiler(String instanceName)
    {
        this.instanceName = instanceName;
        reset();
    }   //TrcPerformanceTimer

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
     * This method resets the profiler data.
     */
    public synchronized void reset()
    {
        profilePoints.clear();
        loopStartNanoTime = 0;
        minLoopIntervalTime = Long.MAX_VALUE;
        maxLoopIntervalTime = Long.MIN_VALUE;
        totalLoopIntervalTime = 0;
        loopCount = 0;
    }   //reset

    /**
     * This method returns the current time in nano second precision.
     *
     * @return current time in seconds.
     */
    public double getHighPrecisionCurrentTime()
    {
        return TrcTimer.getNanoTime()/1000000000.0;
    }   //getHighPrecisionCurrentTime

    /**
     * This method is called to record the loop start time. It also records the interval time since the last call to
     * recordStartTime. It also checks if the interval time is the minimum or maximum it has seen since last reset.
     */
    public synchronized void recordLoopStartTime()
    {
        long currNanoTime = TrcTimer.getNanoTime();

        if (loopStartNanoTime > 0L)
        {
            long intervalTime = currNanoTime - loopStartNanoTime;

            totalLoopIntervalTime += intervalTime;
            loopCount++;
            if (intervalTime < minLoopIntervalTime)
            {
                minLoopIntervalTime = intervalTime;
            }

            if (intervalTime > maxLoopIntervalTime)
            {
                maxLoopIntervalTime = intervalTime;
            }
        }

        loopStartNanoTime = currNanoTime;
    }   //recordLoopStartTime

    /**
     * This method returns the loop start time stamp in seconds.
     *
     * @return loop start time in seconds.
     */
    public double getLoopStartTime()
    {
        return loopStartNanoTime / 1000000000.0;
    }   //getLoopStartTime

    /**
     * This method records the elapsed time of the specified profile point.
     *
     * @param name specifies the name of the profile point.
     * @param startNanoTime specifies the start nanao time for calculating elapsed time.
     * @param inLoop specifies true if the profile point is inside the loop.
     */
    public synchronized void recordProfilePointElapsedTime(String name, long startNanoTime, boolean inLoop)
    {
        ProfilePoint point = profilePoints.get(name);

        if (point == null)
        {
            // This is a new profile point, add it to the map.
            point = new ProfilePoint(name, inLoop, profilePoints.size());
            profilePoints.put(name, point);
        }
        point.totalElapsedTime += TrcTimer.getNanoTime() - startNanoTime;
    }   //recordProfilePointElapsedTime

    /**
     * This method prints the performance metrics of loop profiler.
     *
     * @param tracer specifies the tracer to be used for printing the performance metrics.
     */
    public void printPerformanceMetrics(TrcDbgTrace tracer)
    {
        StringBuilder sb = new StringBuilder("[" + instanceName + "] Loop Performance Metrics:\n");
        double avgLoopInterval = totalLoopIntervalTime / 1000000000.0 / loopCount;

        sb.append(String.format(
            Locale.US, "Average Loop Interval = %.6fs (freq=%.3fHz, min=%.6fs, max=%.6fs)\n",
            avgLoopInterval, 1.0 / avgLoopInterval, minLoopIntervalTime / 1000000000.0,
            maxLoopIntervalTime / 1000000000.0));
        profilePoints.forEach(
            (name, point) ->
            sb.append(String.format(
                Locale.US, "[%02d] %24s=%.6fs\n",
                point.index, name, point.totalElapsedTime / 1000000000.0 / (point.inLoop? loopCount: 1.0))));
        tracer.traceInfo(instanceName, sb.toString());
    }   //printPerformanceMetrics

}   //class TrcLoopProfiler
