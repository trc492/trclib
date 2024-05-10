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

package trclib.archive;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * This class implements a thread-safe data buffer for recording double values. It provides methods to access data
 * in the buffer. It also provides methods to return minimum, maximum and average of the data in the buffer.
 */
public class TrcDataBuffer
{
    private final String instanceName;
    private final int bufferSize;
    private final ArrayList<Double> bufferedData = new ArrayList<>();

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param bufferSize specifies the maximum buffer size, can be 0 if no maximum limit.
     */
    public TrcDataBuffer(String instanceName, int bufferSize)
    {
        this.instanceName = instanceName;
        this.bufferSize = bufferSize;
    }   //TrcDataBuffer

    /**
     * This method returns the instance name and the data in the buffer.
     *
     * @return instance name and data.
     */
    @Override
    public String toString()
    {
        String str;

        synchronized (bufferedData)
        {
            str = instanceName + "[" + bufferSize + "]=" + Arrays.toString(bufferedData.toArray());
        }

        return str;
    }   //toString

    /**
     * This method clears the buffer.
     */
    public void clear()
    {
        synchronized (bufferedData)
        {
            bufferedData.clear();
        }
    }   //clear

    /**
     * This method adds a value to the end of the buffer.
     *
     * @param value specifies the value to be added to the buffer.
     */
    public void addValue(double value)
    {
        synchronized(bufferedData)
        {
            bufferedData.add(value);
            if (bufferSize > 0 && bufferedData.size() > bufferSize)
            {
                // We have a limit on buffer size and we are exceeding it, remove the data at the beginning.
                bufferedData.remove(0);
            }
        }
    }   //addValue

    /**
     * This method returns the indexed value in the buffer.
     *
     * @param index specifies the buffer index of the value to retrieve.
     *
     * @return indexed value.
     */
    public Double getValue(int index)
    {
        Double value = null;

        synchronized (bufferedData)
        {
            if (index < bufferedData.size())
            {
                value = bufferedData.get(index);
            }
        }

        return value;
    }   //getValue

    /**
     * This method gets the last value in the buffer.
     *
     * @return last value in the buffer, null if the buffer is empty.
     */
    public Double getLastValue()
    {
        Double value = null;

        synchronized (bufferedData)
        {
            if (!bufferedData.isEmpty())
            {
                value = bufferedData.get(bufferedData.size() - 1);
            }
        }

        return value;
    }   //getLastValue

    /**
     * This method returns an array of the buffered values.
     *
     * @return array of buffered data, null if buffer is empty.
     */
    public Double[] getBufferedData()
    {
        Double[] data = null;

        synchronized (bufferedData)
        {
            int arraySize = bufferedData.size();

            if (arraySize > 0)
            {
                data = new Double[arraySize];
                bufferedData.toArray(data);
            }
        }

        return data;
    }   //getBufferedData

    /**
     * This method returns the minimum value in the buffer.
     *
     * @return minimum value in the buffer, null if buffer is empty.
     */
    public Double getMinimumValue()
    {
        Double minValue = null;

        synchronized (bufferedData)
        {
            for (double value : bufferedData)
            {
                if (minValue == null)
                {
                    minValue = value;
                }
                else if (value < minValue)
                {
                    minValue = value;
                }
            }
        }

        return minValue;
    }   //getMinimumValue

    /**
     * This method returns the maximum value in the buffer.
     *
     * @return maximum value in the buffer, null if buffer is empty.
     */
    public Double getMaximumValue()
    {
        Double maxValue = null;

        synchronized (bufferedData)
        {
            for (double value : bufferedData)
            {
                if (maxValue == null)
                {
                    maxValue = value;
                }
                else if (value > maxValue)
                {
                    maxValue = value;
                }
            }
        }

        return maxValue;
    }   //getMaximumValue

    /**
     * This method calculates the average value in the buffer.
     *
     * @return average value calculated.
     */
    public double getAverageValue()
    {
        int numValues = 0;
        double sum = 0.0;

        synchronized (bufferedData)
        {
            for (int i = 0; i < bufferedData.size(); i++)
            {
                sum += bufferedData.get(i);
                numValues++;
            }
        }

        return numValues == 0? 0.0: sum/numValues;
    }   //getAverageValue

}   //class TrcDataBuffer
