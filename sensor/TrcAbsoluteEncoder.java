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

package trclib.sensor;

import java.util.function.DoubleSupplier;

import trclib.dataprocessor.TrcWrapValueConverter;

/**
 * This class implements a generic Absolute Encoder that implements the TrcEncoder interface to allow compatibility
 * to other types of encoders. It extends TrcWrapValueConverter that monitors the crossovers of the encoder value
 * and convert it to a continuous value.
 */
public class TrcAbsoluteEncoder extends TrcWrapValueConverter implements TrcEncoder
{
    private double sign = 1.0;
    private double scale = 1.0;
    private double offset = 0.0;
    private double zeroOffset = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param valueSupplier specifies the method to call to get the encoder value.
     * @param rangeLow specifies the low range of the value.
     * @param rangeHigh specifies the high range of the value.
     */
    public TrcAbsoluteEncoder(String instanceName, DoubleSupplier valueSupplier, double rangeLow, double rangeHigh)
    {
        super(instanceName, valueSupplier, rangeLow, rangeHigh);
    }   //TrcAbsoluteEncoder

    //
    // Implements the TrcEncoder interface.
    //

    /**
     * This method resets the encoder revolution counter.
     */
    @Override
    public void reset()
    {
        super.resetConverter();
    }   //reset

    /**
     * This method returns the raw value of the encoder without the crossover count.
     *
     * @return raw encoder value.
     */
    @Override
    public double getRawPosition()
    {
        return valueSupplier.getAsDouble();
    }   //getRawPosition

    /**
     * This method returns the encoder position adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    @Override
    public double getScaledPosition()
    {
        return sign * ((getContinuousValue() - zeroOffset) * scale + offset);
    }   //getScaledPosition

    /**
     * This method reads the raw encoder velocity in encoder units per second.
     *
     * @return raw encoder velocity in encoder units per second.
     */
    @Override
    public double getRawVelocity()
    {
        throw new UnsupportedOperationException("Absolute encoder does not support velocity.");
    }   //getRawVelocity

    /**
     * This method returns the encoder velocity adjusted by scale.
     *
     * @return encoder velocity adjusted by scale.
     */
    @Override
    public double getScaledVelocity()
    {
        throw new UnsupportedOperationException("Absolute encoder does not support velocity.");
    }   //getScaledVelocity

    /**
     * This method reverses the direction of the encoder.
     *
     * @param inverted specifies true to reverse the encoder direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        sign = inverted ? -1.0 : 1.0;
    }   //setInverted

    /**
     * This method checks if the encoder direction is inverted.
     *
     * @return true if encoder direction is rerversed, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        return sign == -1.0;
    }   //isInverted

    /**
     * This method sets the encoder scale and offset.
     *
     * @param scale specifies the scale value.
     * @param offset specifies the offset value.
     * @param zeroOffset specifies the zero offset for absolute encoder.
     */
    @Override
    public void setScaleAndOffset(double scale, double offset, double zeroOffset)
    {
        this.scale = scale;
        this.offset = offset;
        this.zeroOffset = zeroOffset;
    }   //setScaleAndOffset

}   //class TrcAbsoluteEncoder
