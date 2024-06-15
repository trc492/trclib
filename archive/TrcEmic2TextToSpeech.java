/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.nio.charset.StandardCharsets;

import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcSerialBusDevice;
import trclib.dataprocessor.TrcUtil;

/**
 * This class implements a platform independent Emic2 text to speech device that is connected to a Serial Port.
 * This class should be extended by a platform dependent Emic2 device class that provides the asynchronous access
 * to the serial port the device is connected to.
 */
public abstract class TrcEmic2TextToSpeech
{
    public static final int MIN_VOLUME = -48;
    public static final int MAX_VOLUME = 18;

    public enum Voice
    {
        PerfectPaul(0),
        HugeHarry(1),
        BeautifulBetty(2),
        UppityUrsula(3),
        DoctorDennis(4),
        KitTheKid(5),
        FrailFrank(6),
        RoughRita(7),
        WhisperingWendy(8);

        final int value;

        Voice(int value)
        {
            this.value = value;
        }
    }   //enum Voice

    public enum DemoMsg
    {
        Speaking(0),
        Singing(1),
        Spanish(2);

        final int value;

        DemoMsg(int value)
        {
            this.value = value;
        }
    }   //enum DemoMsg

    public enum Language
    {
        USEnglish(0),
        CastilianSpanish(1),
        LatinSpanish(2);

        final int value;

        Language(int value)
        {
            this.value = value;
        }
    }   //enum Language

    public enum Parser
    {
        DECTalk(0),
        Epson(1);

        final int value;

        Parser(int value)
        {
            this.value = value;
        }
    }   //enum Parser

    /**
     * This method issues an asynchronous read of a text string from the device.
     *
     * @param requestId specifies the ID to identify the request. Can be null if none was provided.
     */
    public abstract void asyncReadString(RequestId requestId);

    /**
     * This method writes the string to the device asynchronously.
     *
     * @param text specifies the text string to be written to the device.
     * @param preemptive specifies true for immediate write without queuing, false otherwise.
     */
    public abstract void asyncWriteString(String text, boolean preemptive);

    /**
     * This is used identify the request type.
     */
    public enum RequestId
    {
        PROMPT,
        CONFIG_MSG,
        VERSION_MSG,
        HELP_MSG
    }   //enum RequestId

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private volatile String configMsg = null;
    private volatile String versionMsg = null;
    private volatile String helpMsg = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcEmic2TextToSpeech(String instanceName)
    {
        tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
    }   //TrcEmic2TextToSpeech

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
     * This method is called to start the device in which it would send a read request for the prompt string.
     */
    public void start()
    {
        asyncReadString(RequestId.PROMPT);
    }   //start

    /**
     * This method speaks the specified message.
     *
     * @param msg specifies the message to be spoken.
     */
    public void speak(String msg)
    {
        asyncWriteString("S" + msg + "\n", false);
        asyncReadString(RequestId.PROMPT);
    }   //speak

    /**
     * This method plays the specified demo message.
     *
     * @param msg specifies the demo message.
     */
    public void playDemoMessage(DemoMsg msg)
    {
        asyncWriteString("D" + msg.value + "\n", false);
        asyncReadString(RequestId.PROMPT);
    }   //playDemoMessage

    /**
     * This method aborts the spoken sentence in progress.
     */
    public void stopPlayback()
    {
        asyncWriteString("X\n", true);
    }   //stopPlayback

    /**
     * This method is called to pause/resume the spoken sentence in progress.
     */
    public void togglePlayback()
    {
        asyncWriteString("Z\n", true);
    }   //togglePlayback

    /**
     * This method selects the spoken voice.
     *
     * @param voice specifies the voice to be used.
     */
    public void selectVoice(Voice voice)
    {
        asyncWriteString("N" + voice.value + "\n", false);
        asyncReadString(RequestId.PROMPT);
        configMsg = null;
    }   //selectVoice

    /**
     * This method sets the speaking volume. Valid value is between -48 to 18.
     *
     * @param vol specifies the speaking volume.
     */
    public void setVolume(int vol)
    {
        if (vol < MIN_VOLUME)
        {
            vol = MIN_VOLUME;
        }
        else if (vol > MAX_VOLUME)
        {
            vol = MAX_VOLUME;
        }

        asyncWriteString("V" + vol + "\n", false);
        asyncReadString(RequestId.PROMPT);
        configMsg = null;
    }   //setVolume

    /**
     * This method sets the speaking volume. Valid value is between 0 and 1.0. 0 for mute and 1.0 for full volume.
     *
     * @param vol specifies the speaking volume.
     */
    public void setVolume(double vol)
    {
        vol = TrcUtil.clipRange(vol);
        setVolume((int)((MAX_VOLUME - MIN_VOLUME)*vol + MIN_VOLUME));
    }   //setVolume

    /**
     * This method sets the speaking rate in words per minute.
     *
     * @param rate specifies the speaking rate.
     */
    public void setSpeakingRate(int rate)
    {
        if (rate < 75 || rate > 600)
        {
            throw new IllegalArgumentException("Invalid speaking rate, must be between 75 to 600 words/min.");
        }

        asyncWriteString("W" + rate + "\n", false);
        asyncReadString(RequestId.PROMPT);
        configMsg = null;
    }   //setSpeakingRate

    /**
     * This method sets the spoken language.
     *
     * @param lang specifies the spoken language.
     */
    public void setLanguage(Language lang)
    {
        asyncWriteString("L" + lang.value + "\n", false);
        asyncReadString(RequestId.PROMPT);
        configMsg = null;
    }   //setLanguage

    /**
     * This method selects the parser that parses the sentence.
     *
     * @param parser specifies the parser to use.
     */
    public void selectParser(Parser parser)
    {
        asyncWriteString("P" + parser.value + "\n", false);
        asyncReadString(RequestId.PROMPT);
        configMsg = null;
    }   //selectParser

    /**
     * This method sets the text-to-speech back to default configuration.
     */
    public void revertDefaultConfig()
    {
        asyncWriteString("R\n", false);
        asyncReadString(RequestId.PROMPT);
        configMsg = null;
    }   //revertDefaultConfig

    /**
     * This method returns the current text-to-speech configuration.
     *
     * @param wait specifies true for synchronous access.
     * @return current configuration string if wait is true, null otherwise.
     */
    public String getCurrentConfig(boolean wait)
    {
        if (configMsg == null)
        {
            asyncWriteString("C\n", false);
            asyncReadString(RequestId.CONFIG_MSG);
            asyncReadString(RequestId.PROMPT);
        }

        if (wait)
        {
            while (configMsg == null)
            {
                Thread.yield();
            }
        }

        return configMsg;
    }   //getCurrentConfig

    /**
     * This method returns the firmware version.
     *
     * @param wait specifies true for synchronous access.
     * @return firmware version string if wait is true, null otherwise.
     */
    public String getVersion(boolean wait)
    {
        if (versionMsg == null)
        {
            asyncWriteString("V\n", false);
            asyncReadString(RequestId.VERSION_MSG);
            asyncReadString(RequestId.PROMPT);
        }

        if (wait)
        {
            while (versionMsg == null)
            {
                Thread.yield();
            }
        }

        return versionMsg;
    }   //getVersion

    /**
     * This method returns the help message.
     *
     * @param wait specifies true for synchronous access.
     * @return help message string if wait is true, null otherwise.
     */
    public String getHelpMessage(boolean wait)
    {
        if (helpMsg == null)
        {
            asyncWriteString("H\n", false);
            asyncReadString(RequestId.HELP_MSG);
            asyncReadString(RequestId.PROMPT);
        }

        if (wait)
        {
            while (helpMsg == null)
            {
                Thread.yield();
            }
        }

        return helpMsg;
    }   //getHelpMessage

    /**
     * This method is called when the read request is completed.
     *
     * @param context specifies the read request.
     */
    protected void notify(Object context)
    {
        TrcSerialBusDevice.Request request = (TrcSerialBusDevice.Request) context;
        String reply = null;

        if (request.readRequest && request.buffer != null)
        {
            reply = new String(request.buffer, StandardCharsets.US_ASCII);
            tracer.traceDebug(instanceName, "reply=<" + reply + ">");
        }

        if (reply != null)
        {
            switch ((RequestId)request.requestId)
            {
                case PROMPT:
                    if (reply.equals("."))
                    {
                        //
                        // There was a pause/unpause command, retry the prompt request.
                        //
                        asyncReadString(RequestId.PROMPT);
                    }
                    break;

                case CONFIG_MSG:
                    configMsg = reply;
                    break;

                case VERSION_MSG:
                    versionMsg = reply;
                    break;

                case HELP_MSG:
                    helpMsg = reply;
                    break;
            }
        }
    }   //notify

}   //class FrcEmic2TextToSpeech
