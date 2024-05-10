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

package trclib.archive;

/**
 * This class implements a platform independent color object. The color object describes color in either RGB or HSV
 * formats. It also provides methods to convert between the two formats.
 */
public class TrcColor
{
    private final double[] rgb;
    private final double[] hsv;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param red specifies the red value (0-255).
     * @param green specifies the green value (0-255).
     * @param blue specifies the blue value (0-255).
     */
    public TrcColor(int red, int green, int blue)
    {
        rgb = new double[3];
        rgb[0] = TrcUtil.clipRange(red, 0, 255)/255.0;
        rgb[1] = TrcUtil.clipRange(green, 0, 255)/255.0;
        rgb[2] = TrcUtil.clipRange(blue, 0, 255)/255.0;
        hsv = rgbToHsv(rgb);
    }   //TrcColor

    /**
     * This method returns the normalized red value.
     *
     * @return normalized red value (0.0-1.0).
     */
    public double getRed()
    {
        return rgb[0];
    }   //getRed

    /**
     * This method returns the normalized green value.
     *
     * @return normalized green value (0.0-1.0).
     */
    public double getGreen()
    {
        return rgb[1];
    }   //getGreen

    /**
     * This method returns the normalized blue value.
     *
     * @return normalized blue value (0.0-1.0).
     */
    public double getBlue()
    {
        return rgb[2];
    }   //getBlue

    /**
     * This method returns the normalized RGB values in an array.
     *
     * @return normalized RGB values (0.0-1.0).
     */
    public double[] getRGB()
    {
        return rgb.clone();
    }   //getRGB

    /**
     * This method returns the hue of the HSV color.
     *
     * @return HSV hue.
     */
    public double getHue()
    {
        return hsv[0];
    }   //getHue

    /**
     * This method returns the saturation of the HSV color.
     *
     * @return HSV saturation.
     */
    public double getSaturation()
    {
        return hsv[1];
    }   //getSaturation

    /**
     * This method returns the value of the HSV color.
     *
     * @return HSV value.
     */
    public double getValue()
    {
        return hsv[2];
    }   //getValue

    /**
     * This method returns the HSV values in an array.
     *
     * @return HSV values.
     */
    public double[] getHSV()
    {
        return hsv.clone();
    }   //getHSV

    /**
     * This method translates the RGB color values into HSV.
     *
     * @return HSV values of the color object as an array of doubles.
     */
    public static double[] rgbToHsv(double ... rgb)
    {
        double minValue = Math.min(Math.min(rgb[0], rgb[1]), rgb[2]);
        double maxValue = Math.max(Math.max(rgb[0], rgb[1]), rgb[2]);
        double range = maxValue - minValue;
        double hue, sat, value;

        value = maxValue;
        if (range < 0.00001)
        {
            hue = sat = 0.0;
        }
        else if (maxValue == 0.0)
        {
            sat = 0.0;
            hue = Double.NaN;
        }
        else
        {
            sat = range/maxValue;

            if (rgb[0] == maxValue)
            {
                hue = (rgb[1] - rgb[2])/range;
            }
            else if (rgb[1] == maxValue)
            {
                hue = 2.0 + (rgb[2] - rgb[0])/range;
            }
            else
            {
                hue = 4.0 + (rgb[0] - rgb[1])/range;
            }

            hue *= 60.0;
            if (hue < 0.0)
            {
                hue += 360.0;
            }
        }

        return new double[] {hue, sat, value};
     }  //rgbToHsv

}   //class TrcColor
