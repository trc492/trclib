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

import trclib.archive.TrcTaskMgr.TaskType;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent RGB light. Typically, this class is extended by a platform dependent
 * RGB light device that can be driven by some sort of control signals. The platform dependent RGB light device must
 * implement the abstract methods required by this class. The abstract methods allow this class to control each light
 * channel ON and OFF.
 */
public abstract class TrcRGBLight
{
    /**
     * This method returns the state of the RED light.
     *
     * @return true if the RED light is ON, false otherwise.
     */
    public abstract boolean getRed();

    /**
     * This method returns the state of the GREEN light.
     *
     * @return true if the GREEN light is ON, false otherwise.
     */
    public abstract boolean getGreen();

    /**
     * This method returns the state of the BLUE light.
     *
     * @return true if the RED light is BLUE, false otherwise.
     */
    public abstract boolean getBlue();

    /**
     * This method sets the RED light ON or OFF.
     *
     * @param enabled specifies true to turn the RED light ON, false to turn it off.
     */
    public abstract void setRed(boolean enabled);

    /**
     * This method sets the GREEN light ON or OFF.
     *
     * @param enabled specifies true to turn the GREEN light ON, false to turn it off.
     */
    public abstract void setGreen(boolean enabled);

    /**
     * This method sets the BLUE light ON or OFF.
     *
     * @param enabled specifies true to turn the BLUE light ON, false to turn it off.
     */
    public abstract void setBlue(boolean enabled);

    public static final int COLOR_BLACK     = 0;
    public static final int COLOR_RED       = (1 << 0);
    public static final int COLOR_GREEN     = (1 << 1);
    public static final int COLOR_BLUE      = (1 << 2);
    public static final int COLOR_YELLOW    = (COLOR_RED | COLOR_GREEN);
    public static final int COLOR_MAGENTA   = (COLOR_RED | COLOR_BLUE);
    public static final int COLOR_CYAN      = (COLOR_GREEN | COLOR_BLUE);
    public static final int COLOR_WHITE     = (COLOR_RED | COLOR_GREEN | COLOR_BLUE);
    public static final int COLOR_MIN_VALUE = COLOR_BLACK;
    public static final int COLOR_MAX_VALUE = COLOR_WHITE;

    public enum RGBColor
    {
        RGB_INVALID(-1),
        RGB_BLACK(COLOR_BLACK),
        RGB_RED(COLOR_RED),
        RGB_GREEN(COLOR_GREEN),
        RGB_YELLOW(COLOR_YELLOW),
        RGB_BLUE(COLOR_BLUE),
        RGB_MAGENTA(COLOR_MAGENTA),
        RGB_CYAN(COLOR_CYAN),
        RGB_WHITE(COLOR_WHITE);

        final int value;

        /**
         * Constructor: Create an instance of the enum object with the given value.
         *
         * @param value specifies the ordinal value of the color.
         */
        RGBColor(int value)
        {
            this.value = value;
        }   //RGBColor

        /**
         * This method returns the color enum with the specified ordinal value.
         *
         * @param value specifies the ordinal value of the color.
         * @return color enum with the specified ordinal value.
         */
        public static RGBColor getColor(int value)
        {
            return values()[value];
        }   //getColor

    }   //enum RGBColor

    private enum State
    {
        TURN_ON,
        TURN_OFF,
        DONE
    }   //enum State

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcTaskMgr.TaskObject rgbLightTaskObj;
    private final TrcStateMachine<State> sm;
    private final TrcTimer timer;
    private final TrcEvent timerEvent;

    private RGBColor color = RGBColor.RGB_BLACK;
    private double onPeriod = 0.0;
    private double offPeriod = 0.0;
    private TrcEvent notifyEvent = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcRGBLight(String instanceName)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        rgbLightTaskObj = TrcTaskMgr.createTask(instanceName + ".rgbLightTask", this::rgbLightTask);
        sm = new TrcStateMachine<>(instanceName);
        timer = new TrcTimer(instanceName);
        timerEvent = new TrcEvent(instanceName + ".timerEvent");
    }   //TrcRGBLight

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
     * This method enables/disables the RGB light task that keeps track of the blinking period of the RGB light.
     *
     * @param enabled specifies true to enable RGB light task, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            rgbLightTaskObj.registerTask(TaskType.POST_PERIODIC_TASK);
        }
        else
        {
            rgbLightTaskObj.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This method returns the current color value of the light.
     *
     * @return current color value.
     */
    private int getColorValue()
    {
        int colorValue = 0;

        if (getRed()) colorValue |= COLOR_RED;
        if (getGreen()) colorValue |= COLOR_GREEN;
        if (getBlue()) colorValue |= COLOR_BLUE;

        tracer.traceDebug(instanceName, "color=0x" + Integer.toHexString(colorValue));

        return colorValue;
    }   //getColorValue

    /**
     * This method returns the current color state of the light.
     *
     * @return current color state.
     */
    public RGBColor getColor()
    {
        RGBColor color;
 
        color = RGBColor.getColor(getColorValue());
        tracer.traceDebug(instanceName, "color=" + color);

        return color;
    }   //getColor

    /**
     * This method sets the RGB light with the specified color value. If the specified color value is zero, the light
     * is turned OFF.
     *
     * @param colorValue specifies the color value.
     */
    private void setColorValue(int colorValue)
    {
        tracer.traceDebug(instanceName, "color=0x" + Integer.toHexString(colorValue));
        setRed((colorValue & COLOR_RED) != 0);
        setGreen((colorValue & COLOR_GREEN) != 0);
        setBlue((colorValue & COLOR_BLUE) != 0);
    }   //setColorValue

    /**
     * This method sets the RGB light with the specified color. If the specified color is BLACK, the light is turned
     * OFF.
     *
     * @param color specifies the color.
     */
    public synchronized void setColor(RGBColor color)
    {
        tracer.traceDebug(instanceName, "color=" + color);
        if (sm.isEnabled())
        {
            timer.cancel();
            sm.stop();
        }

        setColorValue(color.value);
    }   //setColor

    /**
     * This method blinks the RGB light with the specified color for the specified amount of ON time and OFF time.
     *
     * @param color specifies the color.
     * @param onPeriod specifies the period for the light to stay ON.
     * @param offPeriod specifies the period for the light to stay OFF.
     */
    public synchronized void setColor(RGBColor color, double onPeriod, double offPeriod)
    {
        tracer.traceDebug(instanceName, "color=%s,onPeriod=%f,offPeriod=%f", color, onPeriod, offPeriod);
        setColor(RGBColor.RGB_BLACK);
        this.color = color;
        this.onPeriod = onPeriod;
        this.offPeriod = offPeriod;
        this.notifyEvent = null;
        sm.start(State.TURN_ON);
        setTaskEnabled(true);
    }   //setColor

    /**
     * This method sets the RGB light ON with the specified color for the specified amount of time. When the amount
     * of time expired, the specified event is signaled.
     *
     * @param color specifies the color.
     * @param onPeriod specifies the period for the light to stay ON.
     * @param event specifies the event to signal when done.
     */
    public void setColor(RGBColor color, double onPeriod, TrcEvent event)
    {
        setColor(color, onPeriod, 0.0);
        this.notifyEvent = event;
    }   //setColor

    /**
     * This method sets the RGB light ON with the specified color for the specified amount of time.
     * 
     * @param color specifies the color.
     * @param onPeriod specifies the period for the light to stay ON.
     */
    public void setColor(RGBColor color, double onPeriod)
    {
        setColor(color, onPeriod, 0.0);
    }   //setColor

    /**
     * This method is called periodically to execute the RGB light operation.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private synchronized void rgbLightTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        if (sm.isReady())
        {
            State state = sm.getState();

            switch (state)
            {
                case TURN_ON:
                    setColorValue(color.value);
                    if (onPeriod != 0.0)
                    {
                        timer.set(onPeriod, timerEvent);
                        sm.waitForSingleEvent(timerEvent, State.TURN_OFF);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case TURN_OFF:
                    setColorValue(COLOR_BLACK);
                    if (offPeriod != 0.0)
                    {
                        timer.set(offPeriod, timerEvent);
                        sm.waitForSingleEvent(timerEvent, State.TURN_ON);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case DONE:
                    if (notifyEvent != null)
                    {
                        notifyEvent.signal();
                    }
                    sm.stop();
                    setTaskEnabled(false);
                    break;
            }
        }
    }   //rgbLightTask

}   //class TrcRGBLight
