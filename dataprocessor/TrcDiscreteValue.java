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

/**
 * This class implements a discrete value where the value can only be set in the interval of specified increment.
 * It provides methods to set the value up or down by increment. It also provides methods to change the increment
 * by an order of 10, in essence making the increment to specify the significant digit of the value to be changed.
 * This is especially useful for using the buttons of a game controller to change a value without using a keyboard
 * to type in a value.
 */
public class TrcDiscreteValue
{
    private final String instanceName;
    private final double valueMin, valueMax;
    private final double incMin, incMax;
    private double value;
    private double inc;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param valueMin specifies the value's minimum.
     * @param valueMax specifies the value's maximum.
     * @param incMin specifies the increment's minimum.
     * @param incMax specifies the increment's maximum.
     * @param defValue specifies the initial value.
     * @param defInc specifies the initial increment.
     */
    public TrcDiscreteValue(
        String instanceName, double valueMin, double valueMax, double incMin, double incMax, double defValue,
        double defInc)
    {
        this.instanceName = instanceName;
        this.valueMin = valueMin;
        this.valueMax = valueMax;
        this.incMin = incMin;
        this.incMax = incMax;
        this.value = defValue;
        this.inc = defInc;
    }   //TrcDiscreteValue

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
     * This method returns the current value.
     *
     * @return current value.
     */
    public double getValue()
    {
        return value;
    }   //getValue

    /**
     * This method returns the current increment.
     *
     * @return current increment.
     */
    public double getIncrement()
    {
        return inc;
    }   //getIncrement

    /**
     * This method increases the value by the increment amount only if the resulting value does not go above valueMax.
     *
     * @return the resulting value.
     */
    public double upValue()
    {
        if (value + inc <= valueMax)
        {
            value += inc;
        }

        return value;
    }   //upValue

    /**
     * This method decreases the value by the increment amount only if the resulting value does not go below valueMin.
     *
     * @return the resulting value.
     */
    public double downValue()
    {
        if (value - inc >= valueMin)
        {
            value -= inc;
        }

        return value;
    }   //downValue

    /**
     * This method increases the increment by an order of 10 only if the resulting increment does not go above incMax.
     *
     * @return the resulting increment.
     */
    public double upIncrement()
    {
        if (inc * 10.0 <= incMax)
        {
            inc *= 10.0;
        }

        return inc;
    }   //upIncrement

    /**
     * This method decreases the increment by an order of 10 only if the resulting increment does not go below incMin.
     *
     * @return the resulting increment.
     */
    public double downIncrement()
    {
        if (inc / 10.0 >= incMin)
        {
            inc /= 10.0;
        }

        return inc;
    }   //downIncrement

}   //class TrcDiscreteValue
