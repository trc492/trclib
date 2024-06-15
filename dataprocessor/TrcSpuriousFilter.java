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

package trclib.dataprocessor;

import trclib.dataprocessor.TrcFilter;

/**
 * This class implements the Spurious Data filter. It is used for detecting and discarding bogus sensor data.
 * When spurious data is detected, it is discarded and the previous data is returned.
 */
public class TrcSpuriousFilter extends TrcFilter
{
    private final double distanceThreshold;
    private Double prevData;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param distanceThreshold specifies the distance threshold from previous data point to be considered spurious.
     */
    public TrcSpuriousFilter(String instanceName, double distanceThreshold)
    {
        super(instanceName);

        this.distanceThreshold = distanceThreshold;
        reset();
    }   //TrcSpuriousFilter

    //
    // Implements TrcFilter abstract methods.
    //

    /**
     * This method resets the filter.
     */
    @Override
    public void reset()
    {
        prevData = null;
    }   //reset

    /**
     * This method returns the filtered data.
     *
     * @param data specifies the data value to be filtered.
     * @return filtered data.
     */
    @Override
    public double filterData(double data)
    {
        if (prevData != null && Math.abs(data - prevData) >= distanceThreshold)
        {
            tracer.traceWarn(instanceName, "Spurious data detected (data=%f, prev=%f)", data, prevData);
            data = prevData;
        }
        else
        {
            prevData = data;
        }

        return data;
    }   //filterData

}   //class TrcSpuriousFilter
