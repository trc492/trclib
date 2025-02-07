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

package trclib.dataprocessor;

import java.util.function.DoubleSupplier;

import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class is typically used to monitor a sensor value that wraps around and convert it to a continuous value.
 * For example, absolute encoder usually returns a value range between 0.0 and 1.0. When the value crosses over the
 * zero crossing, it jumps to the other extreme end. This doesn't work well for controllers such as PID control.
 * This class will monitors the crossovers and will increment or decrement the crossover count depending on which
 * direction the value is crossing over. It provides a getContinuousValue method to return a continuous value
 * incorporating the crossover count as part of the value. For example, instead of returning a value between 0.0 and
 * 1.0, it will return a value such as 3.56 where 3 is the number of crossovers.
 */
public class TrcWrapValueConverter
{
    protected final String instanceName;
    protected final DoubleSupplier valueSupplier;
    private final double range, threshold;
    private final TrcTaskMgr.TaskObject converterTaskObj;

    private boolean enabled = false;
    private double prevReading = 0.0;
    private int numCrossovers = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param valueSupplier specifies the method to call to get the value.
     * @param rangeLow specifies the low range of the value.
     * @param rangeHigh specifies the high range of the value.
     */
    public TrcWrapValueConverter(String instanceName, DoubleSupplier valueSupplier, double rangeLow, double rangeHigh)
    {
        this.instanceName = instanceName;
        this.valueSupplier = valueSupplier;
        this.range = rangeHigh - rangeLow;
        this.threshold = this.range/2.0;

        converterTaskObj = TrcTaskMgr.createTask(instanceName + ".converterTask", this::converterTask);
    }   //TrcWrapValueConverter

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
     * This method returns the state of the converter task.
     *
     * @return true if converter task is enabled, false otherwise.
     */
    public synchronized boolean isTaskEnabled()
    {
        return enabled;
    }   //isTaskEnabled

    /**
     * This method enables/disables the converter task. It is not automatically enabled when created. You must
     * explicitly call this method to enable the converter.
     *
     * @param enabled specifies true for enabling the converter, disabling it otherwise.
     */
    public synchronized void setTaskEnabled(boolean enabled)
    {
        if (!this.enabled && enabled)
        {
            resetConverter();
            converterTaskObj.registerTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }
        else if (this.enabled && !enabled)
        {
            converterTaskObj.unregisterTask();
        }
        this.enabled = enabled;
    }   //setTaskEnabled

    /**
     * This method resets the converter state.
     */
    public synchronized void resetConverter()
    {
        prevReading = valueSupplier.getAsDouble();
        numCrossovers = 0;
    }   //resetConverter

    /**
     * This method returns a continuous value from the sensor reading.
     *
     * @param reading specifies the sensor reading.
     * @return calculated continuous value.
     */
    private double getContinuousValue(double reading)
    {
        return reading + range * numCrossovers;
    }   //getContinuousValue

    /**
     * This method returns the current continuous value.
     *
     * @return current continuous value.
     */
    public synchronized double getContinuousValue()
    {
        return getContinuousValue(valueSupplier.getAsDouble());
    }   //getContinuousValue

    /**
     * This method is called periodically to check for range crossovers.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private synchronized void converterTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        double currReading = valueSupplier.getAsDouble();

        if (Math.abs(currReading - prevReading) > threshold)
        {
            // Detected crossover.
            if (currReading > prevReading)
            {
                // Crossing over backward.
                numCrossovers--;
//                if (Math.abs(getContinuousValue(currReading) - getContinuousValue(prevReading)) > threshold)
//                {
//                    // Reading cannot jump more than threshold, it must be a glitch, ignore the crossover.
//                    numCrossovers++;
//                }
            }
            else
            {
                // Crossing over forward.
                numCrossovers++;
//                if (Math.abs(getContinuousValue(currReading) - getContinuousValue(prevReading)) > threshold)
//                {
//                    // Reading cannot jump more than threshold, it must be a glitch, ignore the crossover.
//                    numCrossovers--;
//                }
            }
        }
        prevReading = currReading;
    }   //converterTask

}   //class TrcWrapValueConverter
