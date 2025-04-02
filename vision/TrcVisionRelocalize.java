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

import java.util.ArrayList;

import trclib.pathdrive.TrcPose2D;

/**
 * This class allows robot re-localization while it's moving.
 */
public class TrcVisionRelocalize
{
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

    private final ArrayList<TimedPose> timedPoses = new ArrayList<>();

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
            timedPoses.add(new TimedPose(timestamp, pose));
        }
    }   //addTimedPose

    /**
     * This method re-localizes the robot with the given vision timestamp and vision determined robot pose.
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
            int minIndex = -1;
            // Find the robot pose with a timestamp closest to the vision timestamp.
            for (int i = 0; i < timedPoses.size(); i++)
            {
                TimedPose timedPose = timedPoses.get(i);
                double timeDelta = Math.abs(visionTimestamp - timedPose.timestamp);
                if (timeDelta < minTimeDelta)
                {
                    minTimeDelta = timeDelta;
                    minTimeDeltaPose = timedPose;
                    minIndex = i;
                }
            }
            // Determine the relative position from the time of the vision frame to current robot pose.
            // Adjust the vision re-localized pose with the same amount of travel.
            TrcPose2D deltaPose = robotPose.relativeTo(minTimeDeltaPose.pose);
            relocalizedPose = visionPose.addRelativePose(deltaPose);
            for (int i = minIndex; i >= 0; i--)
            {
                timedPoses.remove(i);
            }
        }

        return relocalizedPose;
    }   //getRelocalizedPose

}   //class TrcVisionRelocalize
