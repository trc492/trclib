/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.Locale;

/**
 * This class provides high precision time with nanosecond precision but not necessarily nanosecond resolution
 * (that is, how frequently the value changes). There is no guarantee except that the resolution is at least as
 * good as that of System.currentTimeMillis().
 */
public class TrcHighPrecisionTime
{
    private final String instanceName;
    private volatile long timestampNano;
    private volatile double timestampEpoch;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name of the time object.
     */
    public TrcHighPrecisionTime(String instanceName)
    {
        this.instanceName = instanceName;
        recordTimestamp();
    }   //TrcHighPrecisionTime

    /**
     * This method returns the performance data in string form.
     *
     * @return performance data in string form.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: nanoTime=%d, epochTime=%.6f", instanceName, timestampNano, timestampEpoch);
    }   //toString

    /**
     * This method is called to take a snapshot of the current nano time as the start timestamp and the corresponding
     * current time in milliseconds.
     */
    public synchronized void recordTimestamp()
    {
        timestampNano = System.nanoTime();
        timestampEpoch = System.currentTimeMillis() / 1000.0;
    }   //recordTimestamp

    /**
     * This method returns the elapsed time in seconds since the last recorded timestamp.
     *
     * @return elapsed time from last recorded timestamp in seconds.
     */
    public synchronized double getElapsedTime()
    {
        return (System.nanoTime() - timestampNano) / 1000000000.0;
    }   //getElapsedTime

    /**
     * This method returns the elapsed time in seconds of the specified epoch time from the recorded epoch timestamp.
     *
     * @param epochTime specifies the epoch time to compute the elapsed time from the recorded epoch timestamp.
     * @return elapsed time in seconds from the recorded epoch timestamp.
     */
    public synchronized double getElapsedTime(double epochTime)
    {
        return epochTime - timestampEpoch;
    }   //getElapsedTime

    /**
     * Returns the current time in seconds.  Note that while the unit of time of the return value is in seconds,
     * the precision is in nanoseconds but not necessarily nanosecond resolution (that is, how frequently the value
     * changes). There is no guarantee except that the resolution is at least as good as that of
     * System.currentTimeMillis().
     *
     * @return the time difference in seconds, between the current time and midnight, January 1, 1970 UTC with
     *         nanosecond precision.
     */
    public synchronized double getCurrentTime()
    {
        return timestampEpoch + getElapsedTime();
    }   //getCurrentTime

}   //class TrcHighPrecisionTime
