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

package trclib.sensor;

import trclib.pathdrive.TrcPose2D;

/**
 * This interface specifies a common implementation of a DriveBase Odometry device that keeps track of the localization
 * of the robot.
 */
public interface TrcDriveBaseOdometry
{
    /**
     * This method is called once at the beginning of the INPUT_TASK loop. Odometry device can update their cache
     * at this time.
     */
    void updateCache();

    /**
     * This method resets the DriveBase position.
     */
    void reset();

    /**
     * This method returns the DriveBase position.
     *
     * @return DriveBase position.
     */
    TrcPose2D getPosition();

    /**
     * This method sets the DriveBase position.
     *
     * @param pose specifies the DriveBase position.
     */
    void setPosition(TrcPose2D pose);

    /**
     * This method returns the DriveBase velocity.
     *
     * @return DriveBase velocity.
     */
    TrcPose2D getVelocity();

}   //interface TrcDriveBaseOdometry
