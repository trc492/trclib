/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import trclib.inputoutput.TrcTone;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.robotcore.TrcTaskMgr.TaskType;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;

/**
 * This class implements a song player that can parse a notated song in a string
 * buffer and play the notes on a Tone device.
 */
public class TrcSongPlayer
{
    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcTone tone;
    private final TrcTaskMgr.TaskObject playerTaskObj;
    private final TrcTaskMgr.TaskObject stopTaskObj;
    private TrcSong song = null;
    private double barDuration = 0.0;
    private boolean repeat = false;
    private TrcEvent event = null;

    /**
     * Constructor: Create and initialize an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param tone specifies the Tone player.
     */
    public TrcSongPlayer(String instanceName, TrcTone tone)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.tone = tone;
        playerTaskObj = TrcTaskMgr.createTask(instanceName + ".playerTask", this::playerTask);
        stopTaskObj = TrcTaskMgr.createTask(instanceName + ".stopTask", this::stopTask);
    }   //TrcSongPlayer

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
     * This method enables/disables the player task.
     *
     * @param enabled specifies true to enable, false otherwise.
     */
    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            playerTaskObj.registerTask(TaskType.OUTPUT_TASK);
            stopTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);
        }
        else
        {
            playerTaskObj.unregisterTask();
            stopTaskObj.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This method stops the sound and disables the player task.
     */
    public synchronized void stop()
    {
        tone.stop();
        setTaskEnabled(false);
    }   //stop

    /**
     * This method is called to pause the player.
     */
    public synchronized void pause()
    {
        stop();
    }   //pause

    /**
     * This method is called to resume the player.
     */
    public synchronized void resume()
    {
        setTaskEnabled(true);
    }   //resume

    /**
     * This method rewinds the song back to the beginning.
     */
    public synchronized void rewind()
    {
        if (song != null)
        {
            song.rewind();
        }
    }   //rewind

    /**
     * This method plays the specified song. This method is made private because the parameters repeat and event
     * can only have one or the other but not both. If the song is repeating, it will not fire an event.
     *
     * @param song specifies the song to be played.
     * @param barDuration specifies the bar duration in seconds.
     * @param repeat specifies true to play the song repeatedly, false otherwise.
     * @param pause specifies true to pause the song, false to start it immediately.
     * @param event specifies the event to be notified on song completion.
     */
    private synchronized void playSongWorker(
        TrcSong song, double barDuration, boolean repeat, boolean pause, TrcEvent event)
    {
        tracer.traceDebug(
            instanceName, "song=%s,barDur=%.2f,repeat=%s,pause=%s,event=%s", song, barDuration, repeat, pause, event);
        this.song = song;
        this.barDuration = barDuration;
        this.repeat = repeat;
        this.event = repeat? null: event;
        setTaskEnabled(!pause);
    }   //playSongWorker

    /**
     * This method plays the specified song.
     *
     * @param song specifies the song to be played.
     * @param barDuration specifies the bar duration in seconds.
     * @param repeat specifies true to play the song repeatedly, false otherwise.
     * @param pause specifies true to pause the song, false to start it immediately.
     */
    public void playSong(TrcSong song, double barDuration, boolean repeat, boolean pause)
    {
        playSongWorker(song, barDuration, repeat, pause, null);
    }   //playSong

    /**
     * This method plays the specified song.
     *
     * @param song specifies the song to be played.
     * @param barDuration specifies the bar duration in seconds.
     * @param pause specifies true to pause the song, false to start it immediately.
     * @param event specifies the event to be notified on song completion.
     */
    public void playSong(TrcSong song, double barDuration, boolean pause, TrcEvent event)
    {
        playSongWorker(song, barDuration, false, pause, event);
    }   //playSong

    /**
     * This method plays the specified song.
     *
     * @param song specifies the song to be played.
     * @param barDuration specifies the bar duration in seconds.
     * @param pause specifies true to pause the song, false to start it immediately.
     */
    public void playSong(
        TrcSong song, double barDuration, boolean pause)
    {
        playSongWorker(song, barDuration, false, pause, null);
    }   //playSong

    /**
     * This method plays the specified song.
     *
     * @param song specifies the song to be played.
     * @param barDuration Specifies the bar duration in seconds.
     */
    public void playSong(TrcSong song, double barDuration)
    {
        playSongWorker(song, barDuration, false, false, null);
    }   //playSong

    /**
     * This method parses the note string for frequency and duration and plays it.
     *
     * @param note Specifies the note string in the format:
     *        <note>[#|b]<octave>.<noteType>[+]{.<noteType>[+]}
     *        where <note>     - 'A' through 'G'
     *              #          - sharp
     *              b          - flat
     *              <octave>   - 1 through 8
     *              <noteType> - note type (1: whole, 2: half, 4: quarter, ...)
     *              +          - add half time
     * @param barDuration Specifies the bar duration in seconds.
     * @param volume specifies the volume of the note.
     */
    private void playNote(String note, double barDuration, double volume)
    {
        int dotIndex = note.indexOf('.');

        tracer.traceDebug(instanceName, "note=%s, barDuration=%f, volume=%f", note, barDuration, volume);
        if (dotIndex != -1)
        {
            double noteFreq = parseFrequency(note.substring(0, dotIndex));
            double noteLen = parseDuration(note.substring(dotIndex + 1), barDuration);
            tone.playTone(noteFreq, noteLen, volume);
        }
        else
        {
            throw new IllegalArgumentException("Missing note duration <" + note + ">.");
        }
    }   //playNote

    /**
     * This method parses the note frequency from the note string.
     *
     * @param note Specifies the note string in the format:
     *        <note>[#|b]<octave>
     *        where <note>   - 'A' through 'G'
     *              #        - sharp
     *              b        - flat
     *              <octave> - 1 through 8
     *
     * @return note frequency on success and throws IllegalArgumentException on failure.
     */
    private double parseFrequency(String note)
    {
        double freq;

        if (note.charAt(0) >= 'A' && note.charAt(0) <= 'G')
        {
            //
            // Parse note.
            //
            int noteNum = note.charAt(0) - 'C';

            if (noteNum < 0)
            {
                noteNum = (noteNum + 7)*2 + 3;
            }
            else if (noteNum > 2)
            {
                noteNum = noteNum*2 + 3;
            }
            else
            {
                noteNum = noteNum*2 + 4;
            }

            //
            // Parse sharp or flat.
            //
            int i = 1;
            if (i < note.length())
            {
                if (note.charAt(i) == '#')
                {
                    noteNum++;
                    i++;
                }
                else if (note.charAt(i) == 'b')
                {
                    noteNum--;
                    i++;
                }
            }

            //
            // Parse octave.
            //
            if (i < note.length())
            {
                if ((note.charAt(i) >= '1') && (note.charAt(i) <= '8'))
                {
                    noteNum += (note.charAt(i) - '1') * 12;
                    freq = 440.0 * Math.pow(2.0, (noteNum - 49.0) / 12.0);
                }
                else
                {
                    throw new IllegalArgumentException("Invalid note <" + note + ">, invalid octave.");
                }
            }
            else
            {
                throw new IllegalArgumentException("Invalid note <" + note + ">, missing octave.");
            }
        }
        else if (note.charAt(0) == 'R')
        {
            //
            // Note is a rest.
            //
            freq = 0.0;
        }
        else
        {
            throw new IllegalArgumentException("Invalid note <" + note + ">.");
        }

        return freq;
    }   //parseFrequency

    /**
     * This method parses the note length from the note string.
     *
     * @param note Specifies the note string in the format:
     *        <noteType>[+]{.<noteType>[+]}
     *        where <noteType> - note type (1: whole, 2: half, 4: quarter, ...)
     *              +          - add half time
     * @param barDuration Specifies the bar duration in seconds.
     *
     * @return note length in seconds on success and throws IllegalArgumentException on failure.
     */
    private double parseDuration(String note, double barDuration)
    {
        double noteLen = 0.0;
        int dotIndex;

        while ((dotIndex = note.indexOf('.')) != -1)
        {
            noteLen += parseDuration(note.substring(0, dotIndex), barDuration);
            note = note.substring(dotIndex + 1);
        }

        try
        {
            if (note.charAt(note.length() - 1) == '+')
            {
                //
                // 1.5 time.
                //
                noteLen += (barDuration/Integer.parseInt(note.substring(0, note.length() - 1)))*1.5;
            }
            else
            {
                noteLen += barDuration/Integer.parseInt(note);
            }
        }
        catch (Exception e)
        {
            throw new IllegalArgumentException("Invalid duration <" + note + ">.");
        }

        return noteLen;
    }   //parseDuration

    /**
     * This method parses the dynamics volume.
     *
     * @param notation Specifies the dynamics volume notation.
     * @return note volume in the range of 1.0, or -1.0 if not a valid dynamics notation.
     */
    private double parseDynamicsVolume(String notation)
    {
        final String[] dynamics = {"ppp", "pp", "p", "mp", "mf", "f", "ff", "fff"};
        double vol = -1.0;

        for (int i = 0; i < dynamics.length; i++)
        {
            if (notation.equals(dynamics[i]))
            {
                vol = (i + 3)*0.1;
                break;
            }
        }

        return vol;
    }   //parseDynamicsVolume

    /**
     * This method checks if the notation is a dynamics. If so, parses it and performs the appropriate action.
     *
     * @param notation specifies the notation string.
     * @return true if the notation is a dynamics, false otherwise.
     */
    private boolean parseDynamics(String notation)
    {
        boolean isDynamics = false;

        if (notation.charAt(0) == '<')
        {
            // TODO: handle Crescendo.
            isDynamics = true;
        }
        else if (notation.charAt(0) == '>')
        {
            // TODO: handle Diminuendo.
            isDynamics = true;
        }
        else
        {
            double vol = parseDynamicsVolume(notation);
            //
            // If invalid, skip it assuming it is a future feature we don't handle yet.
            //
            if (vol != -1.0)
            {
                song.setCurrentVolume(vol);
                isDynamics = true;
            }
        }

        return isDynamics;
    }   //parseDynamics

    /**
     * This method parses and performs the notation action. It throws an IllegalArgumentException if it is not a
     * recognized notation.
     *
     * @param notation specifies the notation string.
     */
    private void performNotation(String notation)
    {
        boolean found;

        found = parseDynamics(notation);

        if (!found)
        {
            throw new IllegalArgumentException("Invalid notation <" + notation + ">.");
        }
    }   //performNotation

    /**
     * This method is called periodically to check and play the next note in the song.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private synchronized void playerTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        //
        // Move on to the next note only if the current note has finished playing.
        //
        if (!tone.isPlaying())
        {
            while (true)
            {
                String note = song.getNextNote();

                if (note == null && repeat)
                {
                    //
                    // There is no more note in the song. If we are in repeat mode, rewind the song and play it again.
                    //
                    song.rewind();
                    note = song.getNextNote();
                }

                if (note == null)
                {
                    //
                    // The song has ended, let's signal it and quit.
                    //
                    setTaskEnabled(false);
                    if (event != null)
                    {
                        event.signal();
                    }
                    break;
                }
                else if (note.charAt(0) == '#')
                {
                    //
                    // The note is a notation, perform the action and loop back to process the next one.
                    //
                    performNotation(note.substring(1));
                }
                else
                {
                    //
                    // This is a playable note, play it and exit the loop.
                    //
                    playNote(note, barDuration, song.getCurrentVolume());
                    break;
                }
            }
        }
    }   //playerTask

    /**
     * This method is called when the competition mode is about to end. It stops the player if sound is playing.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void stopTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        stop();
    }   //stopTask

}   //class TrcSongPlayer
