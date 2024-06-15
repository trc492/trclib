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

package trclib.sensor;

import trclib.archive.TrcDbgTrace;
import trclib.timer.TrcElapsedTimer;

/**
 * This class implements a platform independent Digital Input device.
 */
public abstract class TrcDigitalInput
{
    private static final String moduleName = TrcDigitalInput.class.getSimpleName();
    protected static TrcElapsedTimer getInputElapsedTimer = null;

    /**
     * This method is provided by the platform dependent digital input device to check the state of the input.
     *
     * @return true if the digital input is active, false otherwise.
     */
    public abstract boolean getInputState();

    protected final String instanceName;
    private boolean inverted = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcDigitalInput(String instanceName)
    {
        this.instanceName = instanceName;
    }   //TrcDigitalInput

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName + ": active=" + isActive();
    }   //toString

    /**
     * This method inverts the digital input state. It is useful for changing a limit switch from Normal Open to
     * Normal Close, for example.
     *
     * @param inverted specifies true to invert the digital input, false otherwise.
     */
    public void setInverted(boolean inverted)
    {
        this.inverted = inverted;
    }   //setInverted

    /**
     * This method returns the state of the input device.
     *
     * @return true if active, false otherwise.
     */
    public boolean isActive()
    {
        return inverted ^ getInputState();
    }   //isActive

    /**
     * This method enables/disables the elapsed timers for performance monitoring.
     *
     * @param enabled specifies true to enable elapsed timers, false to disable.
     */
    public static void setElapsedTimerEnabled(boolean enabled)
    {
        if (enabled)
        {
            if (getInputElapsedTimer == null)
            {
                getInputElapsedTimer = new TrcElapsedTimer(moduleName + ".getInput", 2.0);
            }
        }
        else
        {
            getInputElapsedTimer = null;
        }
    }   //setElapsedTimerEnabled

    /**
     * This method prints the elapsed time info using the given tracer.
     *
     * @param tracer specifies the tracer to be used to print the info.
     */
    public static void printElapsedTime(TrcDbgTrace tracer)
    {
        if (getInputElapsedTimer != null)
        {
            getInputElapsedTimer.printElapsedTime(tracer);
        }
    }   //printElapsedTime

}   //class TrcDigitalInput
