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

package trclib.driverio;

/**
 * This class implements a platform independent Gobilda Indicator Light device. This class is intended to be extended
 * by a platform dependent device class which provides the abstract methods required by this class.
 */
public abstract class TrcGobildaIndicatorLight extends TrcPriorityIndicator<TrcGobildaIndicatorLight.Pattern>
{
    public enum Color
    {
        Black(0.0),
        Red(0.280),
        Orange(0.313),
        Yellow(0.340),
        Sage(0.420),
        Green(0.500),
        Cyan(0.555),
        Blue(0.611),
        Indigo(0.666),
        Violet(0.722),
        White(1.0);
//        Black(0, 0, 0),
//        Red(255, 0, 0),
//        Green(0, 255, 0),
//        Orange(255, 127, 0),
//        Yellow(255, 255, 0),
//        Blue(0, 0, 255),
//        Violet(255, 0, 255),
//        Cyan(0, 255, 255),
//        White(255, 255, 255);

        public final double value;

        /**
         * Constructor: Creates an enum member.
         *
         * @param value specifies the value of the new member.
         */
        Color(double value)
        {
            this.value = value;
        }   //Color

//        Color(int red, int green, int blue)
//        {
//            float[] hsvValues = {0.0f, 0.0f, 0.0f};
//            android.graphics.Color.RGBToHSV(red & 0xff, green & 0xff, blue & 0xff, hsvValues);
//            this.value = hsvValues[0]/360.0;
//        }   //Color

        /**
         * This method looks up the enum member that matches the given value.
         *
         * @param value specifies the enum member value.
         * @return enum member with a matching value.
         */
        public static Color getPattern(double value)
        {
            for (Color c: Color.values())
            {
                if (value == c.value)
                {
                    return c;
                }
            }

            return null;
        }   //getPattern

    }   //enum Color

    /**
     * This class contains information about an LED pattern. An LED pattern contains a pattern type, an array of colors
     * and a time interval between color changes for running patterns.
     */
    public static class Pattern
    {
        public String name;
        public Color ledColor;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param ledColor specifies the REV Blinkin LED pattern.
         */
        public Pattern(String name, Color ledColor)
        {
            this.name = name;
            this.ledColor = ledColor;
        }   //Pattern

        @Override
        public String toString()
        {
            return name;
        }   //toString

    }   //class Pattern

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcGobildaIndicatorLight(String instanceName)
    {
        super(instanceName);
    }   //TrcGobildaIndicatorLight

}   //class TrcGobildaIndicatorLight
