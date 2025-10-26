/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements a platform independent REV Blinkin device. This class is intended to be extended by a
 * platform dependent device class which provides the abstract methods required by this class.
 */
public abstract class TrcRevBlinkin extends TrcPriorityIndicator
{
    public enum RevLedPattern
    {
        FixedRainbowRainBow(-0.99),
        FixedRainbowParty(-0.97),
        FixedRainbowOcean(-0.95),
        FixedRainbowLave(-0.93),
        FixedRainbowForest(-0.91),
        FixedRainbowGlitter(-0.89),
        FixedConfetti(-0.87),
        FixedShotRed(-0.85),
        FixedShotBlue(-0.83),
        FixedShotWhite(-0.81),
        FixedSinelonRainbow(-0.79),
        FixedSinelonParty(-0.77),
        FixedSinelonOcean(-0.75),
        FixedSinelonLava(-0.73),
        FixedSinelonForest(-0.71),
        FixedBeatsPerMinuteRainbow(-0.69),
        FixedBeatsPerMinuteParty(-0.67),
        FixedBeatsPerMinuteOcean(-0.65),
        FixedBeatsPerMinuteLave(-0.63),
        FixedBeatsPerMinuteForest(-0.61),
        FixedFireMedium(-0.59),
        FixedFireLarge(-0.57),
        FixedTwinklesRainbow(-0.55),
        FixedTwinklesParty(-0.53),
        FixedTwinklesOcean(-0.51),
        FixedTwinklesLava(-0.49),
        FixedTwinklesForest(-0.47),
        FixedColorWavesRainbow(-0.45),
        FixedColorWavesParty(-0.43),
        FixedColorWavesOcean(-0.41),
        FixedColorWavesLava(-0.39),
        FixedColorWavesForest(-0.37),
        FixedLarsonScannerRed(-0.35),
        FixedLarsonScannerGray(-0.33),
        FixedLightChaseRed(-0.31),
        FixedLightChaseBlue(-0.29),
        FixedLightChaseGray(-0.27),
        FixedHeartbeatRed(-0.25),
        FixedHeartbeatBlue(-0.23),
        FixedHeartbeatWhite(-0.21),
        FixedHeartbeatGray(-0.19),
        FixedBreathRed(-0.17),
        FixedBreathBlue(-0.15),
        FixedBreathGray(-0.13),
        FixedStrobeRed(-0.11),
        FixedStrobeBlue(-0.09),
        FixedStrobeGold(-0.07),
        FixedStrobeWhite(-0.05),
        Color1EndToEndBlendToBlack(-0.03),
        Color1LarsonScanner(-0.01),
        Color1LightChase(0.01),
        Color1HeartbeatSlow(0.03),
        Color1HeartbeatMedium(0.05),
        Color1HeartbeatFast(0.07),
        Color1BreathSlow(0.09),
        Color1BreathFast(0.11),
        Color1Shot(0.13),
        Color1Strobe(0.15),
        Color2EndToEndBlendToBlack(0.17),
        Color2LarsonScanner(0.19),
        Color2LightChase(0.21),
        Color2HeartbeatSlow(0.23),
        Color2HeartbeatMedium(0.25),
        Color2HeartbeatFast(0.27),
        Color2BreathSlow(0.29),
        Color2BreathFast(0.31),
        Color2Shot(0.33),
        Color2Strobe(0.35),
        SparkleColor1On2(0.37),
        SparkleColor2On1(0.39),
        GradientColor1And2(0.41),
        BeatsPerMinuteColor1And2(0.43),
        EndToEndBlendColor1To2(0.45),
        EndToEndBlendColor1And2(0.47),
        Color1And2NoBlending(0.49),
        TwinklesColor1And2(0.51),
        ColorWavesColor1And2(0.53),
        SinelonColor1And2(0.55),
        SolidHotPink(0.57),
        SolidDarkRed(0.59),
        SolidRed(0.61),
        SolidRedOrange(0.63),
        SolidOrange(0.65),
        SolidGold(0.67),
        SolidYellow(0.69),
        SolidLawnGreen(0.71),
        SolidLime(0.73),
        SolidDarkGreen(0.75),
        SolidGreen(0.77),
        SolidBlueGreen(0.79),
        SolidAqua(0.81),
        SolidSkyBlue(0.83),
        SolidDarkBlue(0.85),
        SolidBlue(0.87),
        SolidBlueViolet(0.89),
        SolidViolet(0.91),
        SolidWhite(0.93),
        SolidGray(0.95),
        SolidDarkGray(0.97),
        SolidBlack(0.99);

        public final double value;

        /**
         * Constructor: Creates an enum member.
         *
         * @param value specifies the value of the new member.
         */
        RevLedPattern(double value)
        {
            this.value = value;
        }   //RevLedPattern

        /**
         * This method looks up the enum member that matches the given value.
         *
         * @param value specifies the enum member value.
         * @return enum member with a matching value.
         */
        public static RevLedPattern getPattern(double value)
        {
            for (RevLedPattern p: RevLedPattern.values())
            {
                if (value == p.value)
                {
                    return p;
                }
            }

            return null;
        }   //getPattern

    }   //enum RevLedPattern

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcRevBlinkin(String instanceName)
    {
        super(instanceName);
    }   //TrcRevBlinkin

}   //class TrcRevBlinkin
