/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
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
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,g
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package trclib.vision;

import trclib.pathdrive.TrcPose2D;

/**
 * This class allows robot re-localization while it's moving.
 */
public class TrcVisionRelocalize
{
    private static final int DEF_BUFFER_SIZE = 100;

    public static class TimedPose
    {
        double timestamp;
        TrcPose2D pose;

        TimedPose(double timestamp, TrcPose2D pose)
        {
            this.timestamp = timestamp;
            this.pose = pose;
        }   //TimedPose
    }   //class TimedPose

    private final TimedPose[] timedPoses;
    private int index;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param bufferSize specifies the circular buffer size. The buffer should be large enough to contain data to
     *        cover the worst case period of processing a vision frame.
     */
    public TrcVisionRelocalize(int bufferSize)
    {
        timedPoses = new TimedPose[bufferSize];
        for (int i = 0; i < bufferSize; i++)
        {
            timedPoses[i] = null;
        }
        index = 0;
    }   //TrcVisionRelocalize

    /**
     * Constructor: Create an instance of the object.
     */
    public TrcVisionRelocalize()
    {
        this(DEF_BUFFER_SIZE);
    }   //TrcVisionRelocalize

    /**
     * This method adds the current timestamp and robot pose in a history queue.
     *
     * @param timestamp specifies current timestamp, must be the same time base as the vision timestamp.
     * @param pose specifies the current robot pose.
     */
    public void addTimedPose(double timestamp, TrcPose2D pose)
    {
        synchronized (timedPoses)
        {
            timedPoses[index] = new TimedPose(timestamp, pose);
            index = (index + 1)%timedPoses.length;
        }
    }   //addTimedPose

    /**
     * This method calculates the re-localized pose accounting for robot movement with the given vision timestamp
     * and vision determined robot pose.
     *
     * @param visionTimestamp specifies the vision timestamp.
     * @param visionPose specifies the vision determined robot pose at the time of the processed vision frame.
     * @param robotPose specifies the current robot pose.
     * @return re-localized robot pose.
     */
    public TrcPose2D getRelocalizedPose(double visionTimestamp, TrcPose2D visionPose, TrcPose2D robotPose)
    {
        TrcPose2D relocalizedPose = null;

        synchronized (timedPoses)
        {
            double minTimeDelta = Double.MAX_VALUE;
            TimedPose minTimeDeltaPose = null;
            // Find the robot pose with a timestamp closest to the vision timestamp.
            for (int offset = 1; offset < timedPoses.length; offset++)
            {
                // Searching backward from the current index.
                int i = index - offset;
                if (i < 0) i += timedPoses.length;
                if (i == index || timedPoses[i] == null)
                {
                    break;
                }

                double timeDelta = Math.abs(visionTimestamp - timedPoses[i].timestamp);
                if (timeDelta <= minTimeDelta)
                {
                    minTimeDelta = timeDelta;
                    minTimeDeltaPose = timedPoses[i];
                }
                else if (visionTimestamp > timedPoses[i].timestamp)
                {
                    // Safe to stop after crossing visionTimestamp because
                    // |Δt| is now guaranteed to increase
                    break;
                }
            }
            // Determine the relative position from the time of the vision frame to current robot pose.
            // Adjust the vision re-localized pose with the same amount of travel.
            if (minTimeDeltaPose != null)
            {
                TrcPose2D deltaPose = robotPose.relativeTo(minTimeDeltaPose.pose);
                relocalizedPose = visionPose.addRelativePose(deltaPose);
            }
        }

        return relocalizedPose;
    }   //getRelocalizedPose

}   //class TrcVisionRelocalize
