/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib.inputoutput;

import java.util.Arrays;

import trclib.robotcore.TrcColor;

/**
 * This class implements a platform independent Addressable LED device. It is intended to be extended by a platform
 * dependent Addressable LED device to provides platform dependent methods to set the color pattern of the LED strip.
 */
public abstract class TrcAddressableLED extends TrcPriorityIndicator<TrcAddressableLED.Pattern>
{
    /**
     * This class contains information about an LED pattern. An LED pattern contains a pattern type, an array of colors
     * and a time interval between color changes for running patterns.
     */
    public static class Pattern
    {
        public enum Type
        {
            Fixed, Running, FadeInFadeOut
        }   //enum Type

        public String name;
        public Type type;
        public TrcColor[] colorPattern;
        public double runningInterval;

        /**
         * This method is called by all constructors to do common initialization.
         *
         * @param name specifies the name of the pattern.
         * @param type specifies the pattern type.
         * @param colorPattern specifies the color pattern as an array of colors, one for each pixel in the LED strip.
         * @param runningInterval specifies the time interval in seconds between each pattern change.
         */
        private void commonInit(String name, Type type, TrcColor[] colorPattern, double runningInterval)
        {
            this.name = name;
            this.type = type;
            this.colorPattern = colorPattern;
            this.runningInterval = runningInterval;
        }   //commonInit

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param colorPattern specifies the color pattern as an array of colors, one for each pixel in the LED strip.
         * @param type specifies the pattern type.
         * @param runningInterval specifies the time interval in seconds between each pattern change.
         */
        public Pattern(String name, TrcColor[] colorPattern, Type type, double runningInterval)
        {
            commonInit(name, type, colorPattern, runningInterval);
        }   //Pattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param colorPattern specifies the color pattern as an array of colors, one for each pixel in the LED strip.
         */
        public Pattern(String name, TrcColor[] colorPattern)
        {
            commonInit(name, Type.Fixed, colorPattern, 0.0);
        }   //Pattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param color specifies the solid color in the color pattern.
         * @param numLEDs specifies the number of LEDs in the color pattern.
         * @param type specifies the pattern type.
         * @param runningInterval specifies the time interval in seconds between each pattern change.
         */
        public Pattern(String name, TrcColor color, int numLEDs, Type type, double runningInterval)
        {
            TrcColor[] fixedColors = new TrcColor[numLEDs];

            Arrays.fill(fixedColors, color);
            commonInit(name, type, fixedColors, runningInterval);
        }   //Pattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param color specifies the solid color in the color pattern.
         * @param numLEDs specifies the number of LEDs in the color pattern.
         */
        public Pattern(String name, TrcColor color, int numLEDs)
        {
            this(name, color, numLEDs, Type.Fixed, 0.0);
        }   //Pattern

        @Override
        public String toString()
        {
            return name;
        }   //toString

    }   //class Pattern

    public abstract void updateLED(TrcColor[] colorPattern);

    protected final int numLEDs;
    protected Pattern currPattern = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numLEDs specifies the number of LEDs in the strip.
     */
    public TrcAddressableLED(String instanceName, int numLEDs)
    {
        super(instanceName);
        this.numLEDs = numLEDs;
    }   //TrcAddressableLED

    /**
     * This method sets the RGB color for the whole LED strip.
     *
     * @param red   specifies the red value (0-255).
     * @param green specifies the green value (0-255).
     * @param blue  specifies the blue value (0-255).
     */
    public void setRGB(int red, int green, int blue)
    {
        currPattern = new Pattern("WholeLengthColor", new TrcColor(red, green, blue), numLEDs);
        updateLED(currPattern.colorPattern);
    }   //setRGB

    /**
     * This method sets the HSV color for the whole LED strip.
     *
     * @param hue   specifies the hue (0-180).
     * @param sat   specifies the saturation (0-255).
     * @param value specifies the value (0-255).
     */
    public void setHSV(double hue, double sat, double value)
    {
        // TODO: Implement TrcColor.hsvToRgb
        // updateLED(currPattern.colorPattern);
    }   //setHSV

    /**
     * This method sets the color for the whole LED strip.
     *
     * @param color specifies the color.
     */
    public void setColor(TrcColor color)
    {
        currPattern = new Pattern("PartialLengthColor", color, numLEDs);
        updateLED(currPattern.colorPattern);
    }   //setColor

    //
    // Implements TrcPriorityIndicator abstract methods.
    //

    /**
     * This method gets the current set pattern.
     *
     * @return currently set pattern.
     */
    @Override
    public Pattern getPattern()
    {
        return currPattern;
    }   //getPattern

    /**
     * This method sets the pattern to the physical indicator device in a device dependent way.
     *
     * @param pattern specifies the indicator pattern. If null, turn off the indicator pattern.
     */
    @Override
    public void setPattern(Pattern pattern)
    {
        this.currPattern = pattern;
        updateLED(currPattern != null? currPattern.colorPattern: null);
    }   //setPattern

}   //class TrcAddressableLED
