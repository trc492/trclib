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

package trclib.sensor;

/**
 * This interface specifies a common implementation of a generic encoder with which makes different types of encoders
 * compatible with each other.
 */
public interface TrcEncoder
{
    /**
     * This method resets the encoder position.
     */
    void reset();

    /**
     * This method reads the raw encoder position in encoder units (generally encoder counts).
     *
     * @return raw position of the encoder in encoder units.
     */
    double getRawPosition();

    /**
     * This method returns the encoder position adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    double getScaledPosition();

    /**
     * This method reads the raw encoder velocity in encoder units per second.
     *
     * @return raw encoder velocity in encoder units per second.
     */
    double getRawVelocity();

    /**
     * This method returns the encoder velocity adjusted by scale.
     *
     * @return encoder velocity adjusted by scale.
     */
    double getScaledVelocity();

    /**
     * This method reverses the direction of the encoder.
     *
     * @param inverted specifies true to reverse the encoder direction, false otherwise.
     */
    void setInverted(boolean inverted);

    /**
     * This method checks if the encoder direction is inverted.
     *
     * @return true if encoder direction is rerversed, false otherwise.
     */
    boolean isInverted();

    /**
     * This method sets the encoder scale and offset.
     *
     * @param scale specifies the scale value.
     * @param offset specifies the offset value.
     * @param zeroOffset specifies the zero offset for absolute encoder.
     */
    void setScaleAndOffset(double scale, double offset, double zeroOffset);

}   //interface TrcEncoder
