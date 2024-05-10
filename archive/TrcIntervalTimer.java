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

package trclib.archive;

/**
 * This class implements a low cost interval timer that will expire at the specified interval. This is different
 * from TrcTimer in that it doesn't signal any event, thus no monitoring task. The caller is responsible for calling
 * the hasExpired() method periodically to check if the interval timer has expired. Once expired, the interval timer
 * is re-armed at the next interval from the time hasExpired() is called.
 */
public class TrcIntervalTimer
{
    private final String instanceName;
    private final double interval;
    private double expirationTime;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the name to identify this instance of the timer.
     * @param interval specifies the interval of the timer in seconds.
     */
    public TrcIntervalTimer(String instanceName, double interval)
    {
        this.instanceName = instanceName;
        this.interval = interval;
        this.expirationTime = TrcTimer.getCurrentTime();
    }   //TrcIntervalTimer

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
     * This method is called periodically to check if the interval timer has expired. If so, it will re-arm the timer
     * to expire at the next interval from the time this is called.
     *
     * @return true if interval timer has expired, false otherwise.
     */
    public boolean hasExpired()
    {
        double currTime = TrcTimer.getCurrentTime();
        boolean expired = currTime >= expirationTime;

        if (expired)
        {
            expirationTime = currTime + interval;
        }

        return expired;
    }   //hasExpired

}   //class TrcIntervalTimer
