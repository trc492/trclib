/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib.timer;

/**
 * This class implements a performance timer to record elapsed time and interval time of a periodic task. It is a
 * performance monitoring tool. It is especially important for PID controlled loops that the loops are executed at
 * a high enough frequency or they will oscillate wildly. It also records the min and max elapsed/interval time values
 * it has seen since the last reset.
 */
public class TrcPerformanceTimer
{
    private final String instanceName;
    private long startTime;
    private long totalElapsedTime;
    private long minElapsedTime;
    private long maxElapsedTime;
    private int elapsedCount;
    private long totalIntervalTime;
    private long minIntervalTime;
    private long maxIntervalTime;
    private int intervalCount;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the name to identify this instance of the performance timer.
     */
    public TrcPerformanceTimer(String instanceName)
    {
        this.instanceName = instanceName;
        reset();
    }   //TrcPerformanceTimer

    /**
     * This method returns the performance data in string form.
     *
     * @return performance data in string form.
     */
    @Override
    public String toString()
    {
        return instanceName +
               ": avgElapsed=" + getAverageElapsedTime() +
               ", minElapsed=" + getMinElapsedTime() +
               ", maxElapsed=" + getMaxElapsedTime() +
               ", avgInterval=" + getAverageIntervalTime() +
               ", minInterval=" + getMinIntervalTime() +
               ", maxInterval=" + getMaxIntervalTime();
    }   //toString

    /**
     * This method resets the performance data.
     */
    public synchronized void reset()
    {
        startTime = 0L;
        totalElapsedTime = totalIntervalTime = 0L;
        minElapsedTime = minIntervalTime = Long.MAX_VALUE;
        maxElapsedTime = maxIntervalTime = 0L;
        elapsedCount = intervalCount = 0;
    }   //reset

    /**
     * This method is called to record the start time. It also records the interval time since the last call to
     * recordStartTime. It also checks if the interval time is the minimum or maximum it has seen since last reset.
     */
    public synchronized void recordStartTime()
    {
        long currNanoTime = TrcTimer.getNanoTime();

        if (startTime > 0L)
        {
            long intervalTime = currNanoTime - startTime;

            totalIntervalTime += intervalTime;
            intervalCount++;

            if (intervalTime < minIntervalTime)
            {
                minIntervalTime = intervalTime;
            }

            if (intervalTime > maxIntervalTime)
            {
                maxIntervalTime = intervalTime;
            }
        }

        startTime = currNanoTime;
    }   //recordStartTime

    /**
     * This method is called to record the elapsed time since the last start time. It also checks if the elapsed time
     * is the minimum or maximum it has seen since last reset.
     */
    public synchronized void recordEndTime()
    {
        // PerformanceTimer could have been enabled after the recordStartTime call, so it will miss recording startTime.
        // In this case, skip recordEndTime since we don't have a valid startTime.
        if (startTime > 0L)
        {
            long elapsedTime = TrcTimer.getNanoTime() - startTime;

            totalElapsedTime += elapsedTime;
            elapsedCount++;

            if (elapsedTime < minElapsedTime)
            {
                minElapsedTime = elapsedTime;
            }

            if (elapsedTime > maxElapsedTime)
            {
                maxElapsedTime = elapsedTime;
            }
        }
    }   //recordEndTime

    /**
     * This method calculates the average elapsed time so far.
     *
     * @return average elapsed time in seconds.
     */
    public synchronized double getAverageElapsedTime()
    {
        return (totalElapsedTime / 1000000000.0) / elapsedCount;
    }   //getAverageElapsedTime

    /**
     * This method returns the minimum elapsed time since the last reset.
     *
     * @return minimum elapsed time in seconds.
     */
    public synchronized double getMinElapsedTime()
    {
        return minElapsedTime / 1000000000.0;
    }   //getMinElapsedTime

    /**
     * This method returns the maximum elapsed time since the last reset.
     *
     * @return maximum elapsed time in seconds.
     */
    public synchronized double getMaxElapsedTime()
    {
        return maxElapsedTime / 1000000000.0;
    }   //getMaxElapsedTime

    /**
     * This method calculates the average interval time so far.
     *
     * @return average interval time in seconds.
     */
    public synchronized double getAverageIntervalTime()
    {
        return (totalIntervalTime / 1000000000.0) / intervalCount;
    }   //getAverageIntervalTime

    /**
     * This method returns the minimum interval time since the last reset.
     *
     * @return minimum interval time in seconds.
     */
    public synchronized double getMinIntervalTime()
    {
        return minIntervalTime / 1000000000.0;
    }   //getMinIntervalTime

    /**
     * This method returns the maximum interval time since the last reset.
     *
     * @return maximum interval time in seconds.
     */
    public synchronized double getMaxIntervalTime()
    {
        return maxIntervalTime / 1000000000.0;
    }   //getMaxIntervalTime

}   //class TrcPerformanceTimer
