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

/**
 * This class implements a generic preset table for looking up preset values.
 */
public class TrcPresets
{
    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final double presetTolerance;
    private final double[] presets;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the task name.
     * @param presetTolerance specifies the preset tolerance.
     * @param presets specifies the preset value array.
     */
    public TrcPresets(String instanceName, double presetTolerance, double... presets)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.presetTolerance = presetTolerance;
        this.presets = presets;
    }   //TrcPresets

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
     * This method checks if the preset index is within the preset table.
     *
     * @param index specifies the preset table index to check.
     * @return true if the index is within the table.
     */
    public boolean validatePresetIndex(int index)
    {
        return index >= 0 && index < presets.length;
    }   //validatePresetIndex

    /**
     * This method returns the preset value at the specified index.
     *
     * @param index specifies the index into the preset table.
     * @return preset value.
     */
    public double getPresetValue(int index)
    {
        return presets[index];
    }   //getPresetValue

    /**
     * This method determines the next preset index up from the specified current value.
     *
     * @param currValue specifies the current value to check against.
     * @return next preset index up.
     */
    public int nextPresetIndexUp(double currValue)
    {
        int index = -1;
        double value = currValue + presetTolerance;

        for (int i = 0; i < presets.length; i++)
        {
            if (presets[i] > value)
            {
                index = i;
                break;
            }
        }

        if (index == -1)
        {
            index = presets.length - 1;
        }

        return index;
    }   //nextPresetIndexUp

    /**
     * This method determines the next preset index down from the specified current value.
     *
     * @param currValue specifies the current value to check against.
     * @return next preset index down.
     */
    public int nextPresetIndexDown(double currValue)
    {
        int index = -1;
        double value = currValue - presetTolerance;

        for (int i = presets.length - 1; i >= 0; i--)
        {
            if (presets[i] < value)
            {
                index = i;
                break;
            }
        }

        if (index == -1)
        {
            index = 0;
        }

        return index;
    }   //nextPresetIndexDown

}   //class TrcPresets
