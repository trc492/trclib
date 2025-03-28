/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib.pathdrive;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import trclib.dataprocessor.TrcUtil;

/**
 * This class implements a waypoint. A waypoint specifies a point on a path that consists of a relative time step
 * relative to the previous point, the x and y components of its position relative to a reference point, the encoder
 * position, velocity and acceleration at this point.
 */
public class TrcWaypoint
{
    public int index;
    public double timeStep;
    public TrcPose2D pose;
    public double encoderPosition;
    public double velocity;
    public double acceleration;
    public double jerk;
    private boolean simpleWaypoint = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param timeStep     specifies the speed denomination in seconds.
     * @param pose         specifies the pose in the path reference frame.
     * @param position     specifies the encoder position at this points. (arc length)
     * @param velocity     specifies the tangential velocity of the wheel at this point.
     * @param acceleration specifies the tangential acceleration at this point.
     * @param jerk         specifies the tangential jerk at this point.
     */
    public TrcWaypoint(
        double timeStep, TrcPose2D pose, double position, double velocity, double acceleration, double jerk)
    {
        this.index = -1;
        this.timeStep = timeStep;
        this.pose = pose;
        this.encoderPosition = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
    }   //TrcWaypoint

    /**
     * Constructor: Create an instance of the object.
     *
     * @param timeStep     specifies the speed denomination in seconds.
     * @param x            specifies the x position in the path reference frame.
     * @param y            specifies the y position in the path reference frame.
     * @param heading      specifies the heading of the robot at this point.
     * @param position     specifies the encoder position at this points. (arc length)
     * @param velocity     specifies the tangential velocity of the wheel at this point.
     * @param acceleration specifies the tangential acceleration at this point.
     * @param jerk         specifies the tangential jerk at this point.
     */
    public TrcWaypoint(
            double timeStep, double x, double y, double heading, double position, double velocity,
            double acceleration, double jerk)
    {
        this(timeStep, new TrcPose2D(x, y, heading), position, velocity, acceleration, jerk);
    }   //TrcWaypoint

    /**
     * Copy constructor: Create a copy of the given object.
     *
     * @param other specifies the other object to be copied.
     */
    public TrcWaypoint(TrcWaypoint other)
    {
        this.index = other.index;
        this.timeStep = other.timeStep;
        this.pose = other.pose;
        this.encoderPosition = other.encoderPosition;
        this.velocity = other.velocity;
        this.acceleration = other.acceleration;
        this.jerk = other.jerk;
        this.simpleWaypoint = other.simpleWaypoint;
    }   //TrcWaypoint

    /**
     * Constructor: Create an instance of the object.
     *
     * @param data specifies an array with 8 elements: timeStep, x, y, heading pos, vel, accel and jerk.
     * @throws ArrayIndexOutOfBoundsException if array size is less than 8.
     */
    public TrcWaypoint(double[] data)
    {
        this(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    }   //TrcWaypoint

    /**
     * Constructor: Create an instance of the object from a given 2D pose.
     *
     * @param pose specifies the position of the way point.
     * @param velocity specifies the velocity of the way point.
     */
    public TrcWaypoint(TrcPose2D pose, TrcPose2D velocity)
    {
        this(0.0, pose, 0.0, velocity != null? TrcUtil.magnitude(velocity.x, velocity.y): 0.0, 0.0, 0.0);
        simpleWaypoint = true;
    }   //TrcWaypoint

    /**
     * Constructor: Create an instance of the object from a given 2D pose.
     *
     * @param pose specifies the position of the way point.
     * @param velocity specifies the velocity of the way point.
     */
    public TrcWaypoint(TrcPose2D pose, double velocity)
    {
        this(0.0, pose, 0.0, velocity, 0.0, 0.0);
        simpleWaypoint = true;
    }   //TrcWaypoint

    /**
     * Constructor: Create an instance of the object from a given 2D pose.
     *
     * @param x specifies the x position in the path reference frame.
     * @param y specifies the y position in the path reference frame.
     * @param heading specifies the heading of the robot at this point.
     * @param velocity specifies the tangential velocity of the wheel at this point.
     */
    public TrcWaypoint(double x, double y, double heading, double velocity)
    {
        this(0.0, new TrcPose2D(x, y, heading), 0.0, velocity, 0.0, 0.0);
        simpleWaypoint = true;
    }   //TrcWaypoint

    /**
     * This method returns the string containing the information of the waypoint.
     *
     * @return waypoint string.
     */
    @Override
    public String toString()
    {
        if (!simpleWaypoint)
        {
            return "Waypoint(simple=" + simpleWaypoint +
                   ",index=" + index +
                   ",timestep=" + timeStep +
                   ",pose=" + pose +
                   ",vel=" + velocity +
                   ",encPos=" + encoderPosition +
                   ",accel=" + acceleration +
                   ",jerk=" + jerk + ")";
        }
        else
        {
            return "Waypoint(simple=" + simpleWaypoint +
                   ",index=" + index +
                   ",pose=" + pose +
                   ",vel=" + velocity + ")";
        }
    }   //toString

    @Override
    public TrcWaypoint clone()
    {
        return new TrcWaypoint(this);
    }   //clone

    /**
     * This method loads waypoint data from a CSV file either on the external file system or attached resources.
     *
     * @param path specifies the file system path or resource name.
     * @param loadFromResources specifies true if the data is from attached resources, false if from file system.
     * @return an array of waypoints.
     */
    public static TrcWaypoint[] loadPointsFromCsv(String path, boolean loadFromResources)
    {
        TrcWaypoint[] waypoints;

        if (!path.endsWith(".csv"))
        {
            throw new IllegalArgumentException(path + " is not a csv file!");
        }

        try
        {
            BufferedReader in = new BufferedReader(
                loadFromResources?
                    new InputStreamReader(
                        Objects.requireNonNull(TrcWaypoint.class.getClassLoader()).getResourceAsStream(path)):
                    new FileReader(path));
            List<TrcWaypoint> pointList = new ArrayList<>();
            String line;

            in.readLine();  // Get rid of the first header line
            while ((line = in.readLine()) != null)
            {
                String[] tokens = line.split(",");

                if (tokens.length != 8)
                {
                    throw new IllegalArgumentException("There must be 8 columns in the csv file!");
                }

                double[] elements = new double[tokens.length];

                for (int i = 0; i < elements.length; i++)
                {
                    elements[i] = Double.parseDouble(tokens[i]);
                }

                TrcWaypoint point = new TrcWaypoint(
                    elements[0], elements[1], elements[2], elements[3], elements[4], elements[5], elements[6],
                    elements[7]);
                pointList.add(point);
            }
            in.close();

            waypoints = pointList.toArray(new TrcWaypoint[0]);
        }
        catch (IOException e)
        {
            throw new RuntimeException(e);
        }

        return waypoints;
    }   //loadPointsFromCsv

    public TrcPose2D getPositionPose()
    {
        // Codereview: should pose be cloned? I don't think so.
        return pose;
    }   //getPositionPose

    /**
     * This method calculates the distance between this waypoint and the other waypoint.
     *
     * @param point specifies the other waypoint.
     * @return distance to the other waypoint.
     */
    public double distanceTo(TrcWaypoint point)
    {
        return TrcUtil.magnitude(point.pose.x - this.pose.x, point.pose.y - this.pose.y);
    }   //distanceTo

}   //class TrcWaypoint
