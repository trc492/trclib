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

package trclib.driverio;

import trclib.robotcore.TrcRobot;
import trclib.dataprocessor.TrcUtil;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements the platform independent game controller and is extended by a platform dependent game
 * controller class providing methods to read various game controls. It also provides monitoring of the controller
 * buttons. If the caller of this class provides a button notification handler, it will call it when there are
 * button events.
 */
public abstract class TrcGameController
{
    /**
     * This enum specifies different drive modes.
     */
    public enum DriveMode
    {
        TankMode,
        ArcadeMode,
        HolonomicMode
    }   //enum DriveMode

    /**
     * This method returns the buttons state of the game controller.
     *
     * @return buttons state of the game controller.
     */
    public abstract int getButtons();

    /**
     * This method is called when a button event is detected.
     *
     * @param buttonValue specifies the value of the button that generated the event.
     * @param pressed specifies true if the button was pressed, false if released.
     */
    protected abstract void notifyButtonEvent(int buttonValue, boolean pressed);

    private static final double DEF_SAMPLING_PERIOD = 0.02;     //Sampling at 50Hz.

    public final TrcDbgTrace tracer;
    protected final String instanceName;
    private final double deadbandThreshold;
    private final TrcTaskMgr.TaskObject buttonEventTaskObj;

    private int prevButtons = 0;
    private double samplingPeriod = DEF_SAMPLING_PERIOD;
    private double nextPeriod = 0.0;
    private int exponent = 2;
    private boolean buttonEventEnabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param deadbandThreshold specifies the deadband of the game controller analog sticks.
     */
    public TrcGameController(String instanceName, double deadbandThreshold)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.deadbandThreshold = deadbandThreshold;
        buttonEventTaskObj = TrcTaskMgr.createTask(instanceName + ".buttonEventTask", this::buttonEventTask);
        prevButtons = getButtons();
    }   //TrcGameController

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
     * This method sets the joystick button sampling period. By default, it is sampling at 50Hz. One could change
     * the sampling period by calling this method.
     *
     * @param period specifies the new sampling period in seconds.
     */
    public void setSamplingPeriod(double period)
    {
        samplingPeriod = period;
    }   //setSamplingPeriod

    /**
     * This method sets the exponential value for raising analog control values.
     *
     * @param exponent sepecifies the exponent value used to raise analog control values.
     */
    public void setExponent(int exponent)
    {
        this.exponent = exponent;
    }   //setExponent

    /**
     * This method enables/disables button event notification.
     *
     * @param enabled specifies true to enable button event notification, false to disable.
     */
    protected void setButtonEventEnabled(boolean enabled)
    {
        if (!buttonEventEnabled && enabled)
        {
            // Enabling button event notification.
            buttonEventTaskObj.registerTask(TrcTaskMgr.TaskType.PRE_PERIODIC_TASK);
        }
        else if (buttonEventEnabled && !enabled)
        {
            // Disabling button event notification.
            buttonEventTaskObj.unregisterTask();
        }
        buttonEventEnabled = enabled;
    }   //setButtonEventEnabled

    /**
     * This method adjusts the analog control value by raising it exponentially and adjusting the sign if appropriate.
     *
     * @param value specifies the analog control value.
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     * @return adjusted analog control value.
     */
    protected double adjustAnalogControl(double value, boolean doExp)
    {
        // Apply deadband.
        value = (Math.abs(value) >= deadbandThreshold)? value: 0.0;
        // Apply exponent curve adjustment.
        value = expValue(value, doExp);
        return value;
    }   //adjustAnalogControl

    /**
     * This method adjusts the analog control curve by using the cubic polynomial: coeff*value^3 + (1 - coeff)*value.
     *
     * @param value specifies the analog control value.
     * @param cubicCoefficient specifies the cubic coefficient.
     */
    protected double adjustAnalogControl(double value, double cubicCoefficient)
    {
        value = (Math.abs(value) >= deadbandThreshold)? value: 0.0;
        value = cubicCoefficient*Math.pow(value, 3) + (1 - cubicCoefficient)*value;
        return value;
    }   //adjustAnalogControl

    /**
     * This method returns the stick direction in radians combining the x and y axes.
     *
     * @param xValue specifies the x-axis value.
     * @param yValue specifies the y-axis value.
     *
     * @return stick direction in radians.
     */
    public double getDirectionRadians(double xValue, double yValue)
    {
        return Math.atan2(yValue, xValue);
    }   //getDirectionRadians

    /**
     * This method returns the stick direction in degrees combining the x and y axes.
     *
     * @param xValue specifies the x-axis value.
     * @param yValue specifies the y-axis value.
     *
     * @return stick direction in degrees.
     */
    public double getDirectionDegrees(double xValue, double yValue)
    {
        return Math.toDegrees(getDirectionRadians(xValue, yValue));
    }   //getDirectionDegrees

    /**
     * This method returns the magnitude value combining the x and y values. The magnitude is calculated by squaring
     * both x and y, sum them and take the square root.
     *
     * @param x specifies the x value.
     * @param y specifies the y value.
     * @return returns the magnitude value.
     */
    public double getMagnitude(double x, double y)
    {
        return TrcUtil.magnitude(x, y);
    }   //getMagnitude

    /**
     * This method returns the exponentially raised of the given value.
     *
     * @param value specifies the value to be raised exponentially.
     * @param doExp specifies true if the value will be exponentially raised, false otherwise.
     * @return exponentially raised value.
     */
    private double expValue(double value, boolean doExp)
    {
        double output;

        if (doExp)
        {
            double sign = Math.signum(value);
            value = Math.abs(value);
            output = Math.pow(value, exponent)*sign;
        }
        else
        {
            output = value;
        }

        return output;
    }   //expValue

    /**
     * This method runs periodically and checks for changes in the button states. If any button changed state,
     * the button handler is called if one exists.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the current robot run mode.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void buttonEventTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        // Button events are human input. They are slow so don't need to run in a fast loop.
        if (slowPeriodicLoop)
        {
            double currTime = TrcTimer.getCurrentTime();
            if (currTime >= nextPeriod)
            {
                int currButtons = getButtons();
                nextPeriod = currTime + samplingPeriod;
                if (buttonEventEnabled && runMode != TrcRobot.RunMode.DISABLED_MODE)
                {
                    int changedButtons = prevButtons ^ currButtons;
                    while (changedButtons != 0)
                    {
                        // buttonMask contains the least significant set bit.
                        int buttonMask = changedButtons & ~(changedButtons ^ -changedButtons);
                        boolean pressed = (currButtons & buttonMask) != 0;
                        int buttonValue = TrcUtil.leastSignificantSetBitPosition(buttonMask);
                        tracer.traceDebug(instanceName, "Button=" + buttonValue + ", pressed=" + pressed);
                        notifyButtonEvent(buttonMask, pressed);
                        // Clear the least significant set bit.
                        changedButtons &= ~buttonMask;
                    }
                }
                prevButtons = currButtons;
            }
        }
    }   //buttonEventTask

}   //class TrcGameController
