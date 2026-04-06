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

package trclib.driverio;

import java.util.Arrays;
import java.util.Locale;

import trclib.dataprocessor.TrcColor;
import trclib.dataprocessor.TrcUtil;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent Addressable LED device. It is intended to be extended by a platform
 * dependent Addressable LED device to provides platform dependent methods to set the color pattern of the LED strip.
 */
public abstract class TrcAddressableLED extends TrcPriorityIndicator
{
    /**
     * This class contains information about an LED pattern. An LED pattern contains a pattern type, an array of colors
     * and a time interval between color changes for running patterns.
     */
    public static class LedPattern
    {
        public enum Type
        {
            Fixed,
            Running,
            FadeInFadeOut,
            LarsonScanner,
            ColorWaves,
            Breath,
            LightChase,
            Heartbeat
        }   //enum Type

        public String name;
        public Type type;
        public TrcColor[] colorPattern;
        public double runningInterval;
        public TrcColor secondaryColor;
        public int patternSize;
        public int chaseSpacing;
        public boolean bidirectional;
        public double hueSpan;
        public double speed;

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
            this.secondaryColor = null;
            this.patternSize = 1;
            this.chaseSpacing = 3;
            this.bidirectional = true;
            this.hueSpan = 180.0;
            this.speed = 1.0;
        }   //commonInit

        public LedPattern setSecondaryColor(TrcColor color)
        {
            this.secondaryColor = color;
            return this;
        }   //setSecondaryColor

        public LedPattern setPatternSize(int patternSize)
        {
            this.patternSize = patternSize;
            return this;
        }   //setPatternSize

        public LedPattern setChaseSpacing(int chaseSpacing)
        {
            this.chaseSpacing = chaseSpacing;
            return this;
        }   //setChaseSpacing

        public LedPattern setBidirectional(boolean bidirectional)
        {
            this.bidirectional = bidirectional;
            return this;
        }   //setBidirectional

        public LedPattern setHueSpan(double hueSpan)
        {
            this.hueSpan = hueSpan;
            return this;
        }   //setHueSpan

        public LedPattern setSpeed(double speed)
        {
            this.speed = speed;
            return this;
        }   //setSpeed

        public static LedPattern createLarsonScanner(
            String name, TrcColor scanColor, TrcColor backgroundColor, int numLEDs, int width, double interval)
        {
            return new LedPattern(name, scanColor, numLEDs, Type.LarsonScanner, interval)
                .setSecondaryColor(backgroundColor)
                .setPatternSize(width)
                .setBidirectional(true);
        }   //createLarsonScanner

        public static LedPattern createColorWaves(
            String name, TrcColor baseColor, int numLEDs, double hueSpan, double interval, double speed)
        {
            return new LedPattern(name, baseColor, numLEDs, Type.ColorWaves, interval)
                .setHueSpan(hueSpan)
                .setSpeed(speed);
        }   //createColorWaves

        public static LedPattern createBreath(String name, TrcColor color, int numLEDs, double cyclePeriod)
        {
            return new LedPattern(name, color, numLEDs, Type.Breath, cyclePeriod);
        }   //createBreath

        public static LedPattern createLightChase(
            String name, TrcColor chaseColor, TrcColor backgroundColor, int numLEDs,
            int width, int spacing, double interval)
        {
            return new LedPattern(name, chaseColor, numLEDs, Type.LightChase, interval)
                .setSecondaryColor(backgroundColor)
                .setPatternSize(width)
                .setChaseSpacing(spacing);
        }   //createLightChase

        public static LedPattern createHeartbeat(String name, TrcColor color, int numLEDs, double cyclePeriod)
        {
            return new LedPattern(name, color, numLEDs, Type.Heartbeat, cyclePeriod);
        }   //createHeartbeat

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param colorPattern specifies the color pattern as an array of colors, one for each pixel in the LED strip.
         * @param type specifies the pattern type.
         * @param runningInterval specifies the time interval in seconds between each pattern change.
         */
        public LedPattern(String name, TrcColor[] colorPattern, Type type, double runningInterval)
        {
            commonInit(name, type, colorPattern, runningInterval);
        }   //LedPattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param colorPattern specifies the color pattern as an array of colors, one for each pixel in the LED strip.
         */
        public LedPattern(String name, TrcColor[] colorPattern)
        {
            commonInit(name, Type.Fixed, colorPattern, 0.0);
        }   //LedPattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param color specifies the solid color in the color pattern.
         * @param numLEDs specifies the number of LEDs in the color pattern.
         * @param type specifies the pattern type.
         * @param runningInterval specifies the time interval in seconds between each pattern change.
         */
        public LedPattern(String name, TrcColor color, int numLEDs, Type type, double runningInterval)
        {
            TrcColor[] fixedColors = new TrcColor[numLEDs];

            Arrays.fill(fixedColors, color);
            commonInit(name, type, fixedColors, runningInterval);
        }   //LedPattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param color specifies the solid color in the color pattern.
         * @param numLEDs specifies the number of LEDs in the color pattern.
         */
        public LedPattern(String name, TrcColor color, int numLEDs)
        {
            this(name, color, numLEDs, Type.Fixed, 0.0);
        }   //LedPattern

        @Override
        public String toString()
        {
            return "(" + name + ":" + type + ")";
        }   //toString

    }   //class LedPattern

    public abstract void updateLED(TrcColor[] colorPattern);

    private static final double DEFAULT_ANIMATION_INTERVAL = 0.05;
    private static final TrcColor OFF_COLOR = new TrcColor(0, 0, 0);

    protected final int numLEDs;
    protected Pattern currPattern = null;
    private final TrcTaskMgr.TaskObject ledTaskObj;
    private double nextLedTaskRunTime = 0.0;
    private double patternStartTime = 0.0;
    private int runningOffset = 0;
    private int scannerPos = 0;
    private int scannerDir = 1;
    private int chaseOffset = 0;

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
        ledTaskObj = TrcTaskMgr.createTask(instanceName + ".ledTask", this::ledTask);
    }   //TrcAddressableLED

    private void setLedTaskEnabled(boolean enabled)
    {
        boolean taskActive = ledTaskObj.isRegistered();

        if (enabled && !taskActive)
        {
            ledTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
            nextLedTaskRunTime = TrcTimer.getCurrentTime();
        }
        else if (!enabled && taskActive)
        {
            ledTaskObj.unregisterTask();
        }
    }   //setLedTaskEnabled

    private void resetAnimationState()
    {
        patternStartTime = TrcTimer.getCurrentTime();
        runningOffset = 0;
        scannerPos = 0;
        scannerDir = 1;
        chaseOffset = 0;
        nextLedTaskRunTime = patternStartTime;
    }   //resetAnimationState

    private boolean isAnimatedPattern(LedPattern ledPattern)
    {
        return ledPattern != null && ledPattern.type != LedPattern.Type.Fixed;
    }   //isAnimatedPattern

    private double getAnimationInterval(LedPattern ledPattern)
    {
        return ledPattern != null && ledPattern.runningInterval > 0.0?
            ledPattern.runningInterval: DEFAULT_ANIMATION_INTERVAL;
    }   //getAnimationInterval

    private TrcColor getPrimaryColor(LedPattern ledPattern)
    {
        return ledPattern != null && ledPattern.colorPattern != null && ledPattern.colorPattern.length > 0?
            ledPattern.colorPattern[0]: OFF_COLOR;
    }   //getPrimaryColor

    private TrcColor getSecondaryColor(LedPattern ledPattern)
    {
        return ledPattern != null && ledPattern.secondaryColor != null? ledPattern.secondaryColor: OFF_COLOR;
    }   //getSecondaryColor

    private TrcColor scaleColor(TrcColor color, double brightness)
    {
        brightness = TrcUtil.clipRange(brightness, 0.0, 1.0);
        return new TrcColor(
            (int) Math.round(color.getRed()*255.0*brightness),
            (int) Math.round(color.getGreen()*255.0*brightness),
            (int) Math.round(color.getBlue()*255.0*brightness));
    }   //scaleColor

    private void fillPattern(TrcColor[] pattern, TrcColor color)
    {
        Arrays.fill(pattern, color);
    }   //fillPattern

    private void renderPattern(LedPattern ledPattern, double currTime)
    {
        TrcColor[] renderColors;

        if (ledPattern == null || numLEDs <= 0)
        {
            updateLED(null);
            return;
        }

        if (ledPattern.colorPattern == null || ledPattern.colorPattern.length == 0)
        {
            renderColors = new TrcColor[numLEDs];
            fillPattern(renderColors, OFF_COLOR);
            updateLED(renderColors);
            return;
        }

        switch (ledPattern.type)
        {
            case Fixed:
                renderColors = ledPattern.colorPattern;
                break;

            case Running:
                renderColors = new TrcColor[numLEDs];
                for (int i = 0; i < numLEDs; i++)
                {
                    int srcIndex = (i + runningOffset)%ledPattern.colorPattern.length;
                    renderColors[i] = ledPattern.colorPattern[srcIndex];
                }
                runningOffset = (runningOffset + 1)%ledPattern.colorPattern.length;
                break;

            case FadeInFadeOut:
            case Breath:
            {
                renderColors = new TrcColor[numLEDs];
                TrcColor primary = getPrimaryColor(ledPattern);
                double period = Math.max(0.2, getAnimationInterval(ledPattern));
                double phase = (currTime - patternStartTime)*Math.PI*2.0/period;
                double brightness = 0.5*(1.0 + Math.sin(phase - Math.PI/2.0));
                fillPattern(renderColors, scaleColor(primary, brightness));
                break;
            }

            case LarsonScanner:
            {
                renderColors = new TrcColor[numLEDs];
                TrcColor primary = getPrimaryColor(ledPattern);
                TrcColor secondary = getSecondaryColor(ledPattern);
                int width = Math.max(1, ledPattern.patternSize);

                fillPattern(renderColors, secondary);
                for (int i = 0; i < numLEDs; i++)
                {
                    int distance = Math.abs(i - scannerPos);
                    if (distance < width)
                    {
                        double brightness = 1.0 - (double) distance/width;
                        renderColors[i] = scaleColor(primary, brightness);
                    }
                }

                scannerPos += scannerDir;
                if (scannerPos >= numLEDs)
                {
                    if (ledPattern.bidirectional)
                    {
                        scannerPos = Math.max(0, numLEDs - 2);
                        scannerDir = -1;
                    }
                    else
                    {
                        scannerPos = 0;
                    }
                }
                else if (scannerPos < 0)
                {
                    scannerPos = Math.min(1, numLEDs - 1);
                    scannerDir = 1;
                }
                break;
            }

            case ColorWaves:
            {
                renderColors = new TrcColor[numLEDs];
                TrcColor primary = getPrimaryColor(ledPattern);
                double[] primaryHsv = primary.getHSV();
                double sat = primaryHsv[1];
                double value = primaryHsv[2];
                double baseHue = primaryHsv[0];
                double waveSpeed = Math.max(0.0, ledPattern.speed);
                double huePhase = (currTime - patternStartTime)*360.0*waveSpeed;

                for (int i = 0; i < numLEDs; i++)
                {
                    double pos = numLEDs > 1? (double) i/(numLEDs - 1): 0.0;
                    double hue = baseHue + pos*ledPattern.hueSpan + huePhase;
                    double[] rgb = TrcColor.hsvToRgb(hue, sat, value);
                    renderColors[i] = new TrcColor(
                        (int) Math.round(rgb[0]*255.0),
                        (int) Math.round(rgb[1]*255.0),
                        (int) Math.round(rgb[2]*255.0));
                }
                break;
            }

            case LightChase:
            {
                renderColors = new TrcColor[numLEDs];
                TrcColor primary = getPrimaryColor(ledPattern);
                TrcColor secondary = getSecondaryColor(ledPattern);
                int width = Math.max(1, ledPattern.patternSize);
                int spacing = Math.max(0, ledPattern.chaseSpacing);
                int segment = width + spacing;
                if (segment <= 0)
                {
                    segment = 1;
                }

                for (int i = 0; i < numLEDs; i++)
                {
                    int index = (i + chaseOffset)%segment;
                    renderColors[i] = index < width? primary: secondary;
                }
                chaseOffset = (chaseOffset + 1)%segment;
                break;
            }

            case Heartbeat:
            {
                renderColors = new TrcColor[numLEDs];
                TrcColor primary = getPrimaryColor(ledPattern);
                double period = Math.max(0.4, getAnimationInterval(ledPattern));
                double x = ((currTime - patternStartTime)%period)/period;
                double pulse1 = Math.exp(-Math.pow((x - 0.14)/0.05, 2.0));
                double pulse2 = 0.85*Math.exp(-Math.pow((x - 0.34)/0.08, 2.0));
                double brightness = TrcUtil.clipRange(pulse1 + pulse2, 0.0, 1.0);

                fillPattern(renderColors, scaleColor(primary, brightness));
                break;
            }

            default:
                renderColors = ledPattern.colorPattern;
                break;
        }

        updateLED(renderColors);
    }   //renderPattern

    private void ledTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        Pattern pattern = currPattern;

        if (pattern != null)
        {
            LedPattern ledPattern = (LedPattern) pattern.devPattern;
            double currTime = TrcTimer.getCurrentTime();

            if (!isAnimatedPattern(ledPattern))
            {
                setLedTaskEnabled(false);
            }
            else if (currTime >= nextLedTaskRunTime)
            {
                nextLedTaskRunTime = currTime + getAnimationInterval(ledPattern);
                renderPattern(ledPattern, currTime);
            }
        }
        else
        {
            setLedTaskEnabled(false);
        }
    }   //ledTask

    /**
     * This method sets the color for the whole LED strip.
     *
     * @param color specifies the color.
     */
    public void setColor(TrcColor color)
    {
        String colorName = String.format(
            Locale.US, "RGB_%x",
            ((int) (color.getRed()*255.0)) +
            (((int) (color.getGreen()*255.0)) << 8) +
            (((int) (color.getBlue()*255.0)) << 16));
        LedPattern ledPattern = new LedPattern(colorName, color, numLEDs);
        setPattern(new Pattern("WholeLength:" + colorName, ledPattern));
    }   //setColor

    /**
     * This method sets the RGB color for the whole LED strip.
     *
     * @param red   specifies the red value (0-255).
     * @param green specifies the green value (0-255).
     * @param blue  specifies the blue value (0-255).
     */
    public void setRGB(int red, int green, int blue)
    {
        setColor(new TrcColor(red, green, blue));
    }   //setRGB

    /**
     * This method sets the HSV color for the whole LED strip.
     *
     * @param hue   specifies the hue (0-360), 0-180 is also accepted for legacy callers.
     * @param sat   specifies the saturation (0-255).
     * @param value specifies the value (0-255).
     */
    public void setHSV(int hue, int sat, int value)
    {
        double h = hue <= 180? hue*2.0: hue;
        double s = TrcUtil.clipRange(sat, 0, 255)/255.0;
        double v = TrcUtil.clipRange(value, 0, 255)/255.0;
        double[] rgb = TrcColor.hsvToRgb(h, s, v);

        setRGB(
            (int) Math.round(rgb[0]*255.0),
            (int) Math.round(rgb[1]*255.0),
            (int) Math.round(rgb[2]*255.0));
    }   //setHSV

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
        if (currPattern == null)
        {
            setLedTaskEnabled(false);
            updateLED(null);
        }
        else
        {
            LedPattern ledPattern = (LedPattern) currPattern.devPattern;

            if (isAnimatedPattern(ledPattern))
            {
                resetAnimationState();
                renderPattern(ledPattern, patternStartTime);
                setLedTaskEnabled(true);
            }
            else
            {
                setLedTaskEnabled(false);
                updateLED(ledPattern.colorPattern);
            }
        }
    }   //setPattern

}   //class TrcAddressableLED
