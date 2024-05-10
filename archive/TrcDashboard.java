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
 * This interface provides a platform independent way to display info to the dashboard. It is mainly for TrcLib
 * which is platform agnostic. A platform dependent class will implement methods in this interface.
 */
public abstract class TrcDashboard
{
    /**
     * This method clears all the display lines.
     */
    public abstract void clearDisplay();

    /**
     * This method refresh the display lines on the Driver Station.
     */
    public abstract void refreshDisplay();

    /**
     * This method displays a message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param msg specifies the message string.
     */
    public abstract void displayPrintf(int lineNum, String msg);

    /**
     * This method displays a formatted message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param format  specifies the format string.
     * @param args    specifies variable number of substitution arguments.
     */
    public abstract void displayPrintf(int lineNum, String format, Object... args);

    /**
     * This method returns the value of the named boolean data read from the Telemetry class. If the named data does
     * not exist, it is created and assigned the given default value. Then it is sent to the Driver Station.
     *
     * @param key specifies the name associated with the boolean data.
     * @param defaultValue specifies the default value if it does not exist.
     * @return boolean data value.
     */
    public abstract boolean getBoolean(String key, boolean defaultValue);

    /**
     * This method sets the named boolean data with the given value and also sends it to the Driver Station.
     *
     * @param key specifies the name associated with the boolean data.
     * @param value specifies the data value.
     */
    public abstract void putBoolean(String key, boolean value);

    /**
     * This method returns the value of the named double data read from the Telemetry class. If the named data does
     * not exist, it is created and assigned the given default value. Then it is sent to the Driver Station.
     *
     * @param key specifies the name associated with the double data.
     * @param defaultValue specifies the default value if it does not exist.
     * @return double data value.
     */
    public abstract double getNumber(String key, double defaultValue);

    /**
     * This method sets the named double data with the given value and also sends it to the Driver Station.
     *
     * @param key specifies the name associated with the double data.
     * @param value specifies the data value.
     */
    public abstract void putNumber(String key, double value);

    /**
     * This method returns the value of the named string data read from the Telemetry class. If the named data does
     * not exist, it is created and assigned the given default value. Then it is sent to the Driver Station.
     *
     * @param key specifies the name associated with the string data.
     * @param defaultValue specifies the default value if it does not exist.
     * @return string data value.
     */
    public abstract String getString(String key, String defaultValue);

    /**
     * This method sets the named string data with the given value and also sends it to the Driver Station.
     *
     * @param key specifies the name associated with the string data.
     * @param value specifies the data value.
     */
    public abstract void putString(String key, String value);

    protected static final int MAX_NUM_TEXTLINES = 16;
    protected static final String displayKeyFormat = "%02d";
    protected static TrcDashboard instance = null;
    protected final int numLines;

    /**
     * This method allows any class to get an instance of the dashboard so that it can display information on its
     * display.
     *
     * @return global instance of the dashboard object.
     */
    public static TrcDashboard getInstance()
    {
        return instance;
    }   //getInstance

    /**
     * Constructor: Create an instance of the object.
     *
     * @param numLines specifies the number of display lines.
     */
    public TrcDashboard(int numLines)
    {
        this.numLines = numLines;
    }   //TrcDashboard

    /**
     * This method returns the number of text lines on the dashboard display.
     *
     * @return number of text lines on the dashboard display.
     */
    public int getNumTextLines()
    {
        return numLines;
    }   //getNumTextLines

}   //class TrcDashboard
