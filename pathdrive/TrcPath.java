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

import java.util.Arrays;

import trclib.robotcore.TrcUtil;
import trclib.dataprocessor.TrcWarpSpace;

/**
 * This class implements a path. A path is consists of an array of waypoints, and can be used for path following,
 * such as motion profiling, pure pursuit, etc. Since heading could be in degrees or radians, each path object specifies
 * the units of its heading value, and can be converted. If the timesteps are not specified, velocity and position data
 * can be used to infer approximate timesteps. Paths can be read from CSV files.
 */
public class TrcPath
{
    /**
     * This method loads waypoints from a CSV file and create a path with them.
     *
     * @param inDegrees         specifies true if the heading values are in degrees, false if they are radians.
     * @param path              specifies the file path or the resource name where we load the waypoints.
     * @param loadFromResources specifies true if waypoints are loaded from resources, false if from file path.
     * @return created path with the loaded waypoints.
     */
    public static TrcPath loadPathFromCsv(boolean inDegrees, String path, boolean loadFromResources)
    {
        return new TrcPath(inDegrees, TrcWaypoint.loadPointsFromCsv(path, loadFromResources));
    }   //loadPathFromCsv

    private final TrcWaypoint[] waypoints;
    private boolean inDegrees;

    /**
     * Constructor: Create a new TrcPath object. Must supply at least 2 entries.
     *
     * @param inDegrees specifies true if the heading values are in degrees, false if they are radians.
     * @param waypoints specifies the array of points that will constitute this path. Cannot be null, and must have
     *                  at least 2 waypoints.
     */
    public TrcPath(boolean inDegrees, TrcWaypoint... waypoints)
    {
        if (waypoints == null || waypoints.length <= 1)
        {
            throw new IllegalArgumentException("Waypoints cannot be null or have less than 2 entries!");
        }

        this.inDegrees = inDegrees;
        this.waypoints = waypoints;
    }   //TrcPath

    /**
     * Constructor: Create a new TrcPath object. Must supply at least 2 entries.
     *
     * @param waypoints specifies the array of points that will constitute this path. Cannot be null, and must have
     *                  at least 2 waypoints.
     */
    public TrcPath(TrcWaypoint... waypoints)
    {
        this(true, waypoints);
    }   //TrcPath

    /**
     * This method returns the path info in string form.
     *
     * @return path info in string form.
     */
    @Override
    public String toString()
    {
        return "TrcPath(degrees=" + isInDegrees() + ", " + Arrays.toString(waypoints) + ")";
    }   //toString

    /**
     * Translate the path by an x and y offset.
     *
     * @param x Amount to offset in x axis.
     * @param y Amount to offset in y axis.
     * @return new {@link TrcPath} object which is this path with an offset.
     */
    public TrcPath translate(double x, double y)
    {
        TrcWaypoint[] points = new TrcWaypoint[waypoints.length];
        for (int i = 0; i < waypoints.length; i++)
        {
            TrcWaypoint wp = new TrcWaypoint(waypoints[i]);
            wp.pose.x += x;
            wp.pose.y += y;
            points[i] = wp;
        }
        return new TrcPath(inDegrees, points);
    }   //translate

    /**
     * Insert a waypoint in the middle of the path.
     *
     * @param index    The index to insert the new point, in the range [0, <code>getSize()</code>]
     * @param waypoint The waypoint to insert.
     * @return A new {@link TrcPath} object with the waypoint inserted.
     */
    public TrcPath insertWaypoint(int index, TrcWaypoint waypoint)
    {
        TrcWaypoint[] newPoints = new TrcWaypoint[waypoints.length + 1];
        System.arraycopy(waypoints, 0, newPoints, 0, index);
        newPoints[index] = waypoint;
        System.arraycopy(waypoints, index, newPoints, index + 1, waypoints.length - index);
        return new TrcPath(inDegrees, newPoints);
    }   //insertWaypoint

    /**
     * This method makes a copy of this waypoint.
     *
     * @return a copy of this waypoint.
     */
    public TrcPath clone()
    {
        TrcWaypoint[] points = new TrcWaypoint[waypoints.length];
        for (int i = 0; i < points.length; i++)
        {
            points[i] = waypoints[i].clone();
        }
        return new TrcPath(inDegrees, points);
    }   //clone

    /**
     * This method translates all the waypoints in the path relative to the specified referencePose.
     *
     * @param referencePose specifies the reference pose to translate all waypoints to be referenced to.
     * @return the translated path.
     */
    public TrcPath relativeTo(TrcPose2D referencePose)
    {
        TrcWaypoint[] waypoints = inDegrees ? this.waypoints : toDegrees().waypoints;
        TrcWaypoint[] newPoints = new TrcWaypoint[waypoints.length];

        for (int i = 0; i < waypoints.length; i++)
        {
            TrcWaypoint wp = waypoints[i].clone();
            wp.pose.setAs(wp.pose.relativeTo(referencePose, true));
            newPoints[i] = wp;
        }
        TrcPath path = new TrcPath(true, newPoints);

        return inDegrees ? path : path.toRadians();
    }   //relativeTo

    /**
     * This method returns a path that converts all the waypoints to absolute positions.
     *
     * @param absoluteStartPose specifies the absolute pose of the first point in the path.
     * @return path with absolute waypoints.
     */
    public TrcPath toAbsolute(TrcPose2D absoluteStartPose)
    {
        TrcWaypoint[] waypoints = inDegrees ? this.waypoints : toDegrees().waypoints;
        TrcWaypoint[] newPoints = new TrcWaypoint[waypoints.length];

        for (int i = 0; i < waypoints.length; i++)
        {
            TrcWaypoint wp = waypoints[i].clone();
            wp.pose.setAs(absoluteStartPose.addRelativePose(wp.pose));
            newPoints[i] = wp;
        }
        TrcPath path = new TrcPath(true, newPoints);

        return inDegrees ? path : path.toRadians();
    }   //toAbsolute

    /**
     * This method translates all the waypoints in the path relative to the first waypoint in the path.
     *
     * @return the translated path.
     */
    public TrcPath relativeToStart()
    {
        return relativeTo(waypoints[0].getPositionPose());
    }   //relativeToStart

    /**
     * Set the velocity and accelerations of the waypoints in the path to follow a trapezoidal velocity profile.
     * This is characterized by maxVel and maxAccel. Up to two points will be inserted into the path to make the
     * robot accelerate at maxAccel. Between those two points, the robot velocity will be maxVel.
     *
     * @param maxVel   The maximum velocity of the path, with matching units.
     * @param maxAccel The maximum acceleration of the path, with matching units.
     * @return A new {@link TrcPath} object, with the velocities and accelerations matching the profile.
     */
    public TrcPath trapezoidVelocity(double maxVel, double maxAccel)
    {
        maxVel = Math.abs(maxVel);
        maxAccel = Math.abs(maxAccel);
        TrcPath path = clone();
        for (TrcWaypoint waypoint : path.getAllWaypoints())
        {
            waypoint.velocity = maxVel;
            waypoint.acceleration = 0;
        }
        path.waypoints[0].velocity = 0;
        path.waypoints[0].acceleration = maxAccel;

        // v/t = a, v*t/2 = d (constant acceleration from rest is a triangle)
        // d = v^2/(2*a)
        double dist = Math.pow(maxVel, 2) / (2 * maxAccel); // this is the distance required to get up to speed
        double length = 0;
        // Iterate forwards, ramping up the velocity to maxVel, at the rate maxAccel
        // Stop when the partial arclength exceeds the speed up distance
        // This creates the first "ramp up" of the trapezoid
        for (int i = 0; i < path.getSize() - 1; i++)
        {
            TrcWaypoint from = path.waypoints[i];
            TrcWaypoint to = path.waypoints[i + 1];
            double segLength = from.distanceTo(to);
            length += segLength;
            if (length <= dist)
            {
                to.velocity = Math.sqrt(2 * length * maxAccel);
                to.acceleration = maxAccel;
            }
            else
            {
                double prevDist = length - segLength;
                TrcWaypoint inserted = interpolate(from, to, (dist - prevDist) / segLength);
                inserted.velocity = maxVel;
                inserted.acceleration = 0;
                path = path.insertWaypoint(i + 1, inserted);
                break;
            }
        }

        // End at rest
        path.waypoints[path.getSize() - 1].velocity = 0;
        path.waypoints[path.getSize() - 1].acceleration = 0;
        length = 0;
        // Same as before, but backwards from the end. This creates the "ramp down" of the trapezoid.
        for (int i = path.getSize() - 1; i > 0; i--)
        {
            TrcWaypoint from = path.waypoints[i - 1];
            TrcWaypoint to = path.waypoints[i];
            double segLength = from.distanceTo(to);
            length += segLength;
            if (length <= dist)
            {
                double vel = Math.sqrt(2 * length * maxAccel);
                if (vel >= from.velocity)
                    break;
                from.velocity = vel;
                from.acceleration = -maxAccel;
            }
            else
            {
                double prevDist = length - segLength;
                TrcWaypoint inserted = interpolate(to, from, (dist - prevDist) / segLength);
                inserted.velocity = maxVel;
                inserted.acceleration = -maxAccel;
                path = path.insertWaypoint(i, inserted);
                break;
            }
        }
        path.inferTimeSteps();
        return path;
    }   //trapezoidVelocity

    /**
     * This method returns a waypoint that is interpolated between the two specified waypoints with the specified
     * weight between the points.
     *
     * @param point1 specifies the first waypoint.
     * @param point2 specifies the second waypoint.
     * @param weight specifies the weighted distance between the two points.
     */
    private TrcWaypoint interpolate(TrcWaypoint point1, TrcWaypoint point2, double weight)
    {
        double timestep = interpolate(point1.timeStep, point2.timeStep, weight);
        double x = interpolate(point1.pose.x, point2.pose.x, weight);
        double y = interpolate(point1.pose.y, point2.pose.y, weight);
        double position = interpolate(point1.encoderPosition, point2.encoderPosition, weight);
        double velocity = interpolate(point1.velocity, point2.velocity, weight);
        double acceleration = interpolate(point1.acceleration, point2.acceleration, weight);
        double jerk = interpolate(point1.jerk, point2.jerk, weight);
        double heading = interpolate(point1.pose.angle,
                                     TrcWarpSpace.getOptimizedTarget(point2.pose.angle, point1.pose.angle, inDegrees? 360.0: 2*Math.PI), weight);
        return new TrcWaypoint(timestep, new TrcPose2D(x, y, heading), position, velocity, acceleration, jerk);
    }   //interpolate

    /**
     * This method returns an interpolated value between the two specified values with the specified
     * weight between the two values.
     *
     * @param start specifies the first value.
     * @param end specifies the second value.
     * @param weight specifies the weighted distance between the two values.
     */
    private double interpolate(double start, double end, double weight)
    {
        if (!TrcUtil.inRange(weight, 0.0, 1.0))
        {
            throw new IllegalArgumentException("Weight must be in range [0,1]!");
        }
        return (1.0 - weight) * start + weight * end;
    }   //interpolate

    /**
     * This method returns the last waypoint in the path.
     *
     * @return last waypoint.
     */
    public TrcWaypoint getLastWaypoint()
    {
        return waypoints[waypoints.length - 1];
    }   //getLastWaypoint

    /**
     * This method returns the waypoint at the given index of the path.
     *
     * @param index specifies the index to the path array.
     * @return the waypoint at the given index.
     */
    public TrcWaypoint getWaypoint(int index)
    {
        return waypoints[index];
    }   //getWaypoint

    /**
     * Check if this path defines heading using degrees.
     *
     * @return True if degrees are used, false if radians.
     */
    public boolean isInDegrees()
    {
        return inDegrees;
    }   //isInDegrees

    /**
     * This method returns the number of waypoints in this path.
     *
     * @return the number of waypoints in this path.
     */
    public int getSize()
    {
        return waypoints.length;
    }   //getSize

    /**
     * This method returns the array of waypoints of the path.
     *
     * @return the waypoint array. This is the same instance, so modifying it will affect other users of this path.
     */
    public TrcWaypoint[] getAllWaypoints()
    {
        return waypoints;
    }   //getAllWaypoints

    /**
     * Create a new path identical to this one, except the heading values are in degrees. If this path's headings are
     * already in degrees, just return deep copy.
     *
     * @return A TrcPath object which the same waypoints except all heading values are now in degrees, if they weren't before.
     */
    public TrcPath toDegrees()
    {
        TrcWaypoint[] waypoints = new TrcWaypoint[this.waypoints.length];

        for (int i = 0; i < this.waypoints.length; i++)
        {
            TrcWaypoint waypoint = new TrcWaypoint(this.waypoints[i]);
            // If already in degree mode, don't convert again.
            waypoint.pose.angle = inDegrees ? waypoint.pose.angle : Math.toDegrees(waypoint.pose.angle);
            waypoints[i] = waypoint;
        }

        return new TrcPath(true, waypoints);
    }   //toDegrees

    /**
     * Create a new path identical to this one, except the heading values are in radians. If this path's headings are
     * already in radians, just return deep copy.
     *
     * @return A TrcPath object which the same waypoints except all heading values are now in radians, if they weren't before.
     */
    public TrcPath toRadians()
    {
        TrcWaypoint[] waypoints = new TrcWaypoint[this.waypoints.length];

        for (int i = 0; i < this.waypoints.length; i++)
        {
            TrcWaypoint waypoint = new TrcWaypoint(this.waypoints[i]);
            // If already in radian mode, don't convert again.
            waypoint.pose.angle = inDegrees ? Math.toRadians(waypoint.pose.angle) : waypoint.pose.angle;
            waypoints[i] = waypoint;
        }

        return new TrcPath(false, waypoints);
    }   //toRadians

    /**
     * If not already in degrees, convert this path's heading values to degrees.
     */
    public void mapSelfToDegrees()
    {
        if (!inDegrees)
        {
            inDegrees = true;

            for (TrcWaypoint waypoint : waypoints)
            {
                waypoint.pose.angle = Math.toDegrees(waypoint.pose.angle);
            }
        }
    }   //mapSelfToDegrees

    /**
     * If not already in radians, convert this path's heading values to radians.
     */
    public void mapSelfToRadians()
    {
        if (inDegrees)
        {
            inDegrees = false;

            for (TrcWaypoint waypoint : waypoints)
            {
                waypoint.pose.angle = Math.toRadians(waypoint.pose.angle);
            }
        }
    }   //mapSelfToRadians

    /**
     * Get the estimated duration of the entire path.
     *
     * @return The estimated time duration, in the same units as <code>timeStep</code>.
     */
    public double getPathDuration()
    {
        double duration = 0;

        for (TrcWaypoint waypoint : waypoints)
        {
            duration += waypoint.timeStep;
        }

        return duration;
    }   //getPathDuration

    /**
     * Get the curved length of the entire path.
     *
     * @return The curved length of the entire path, in the same units as <code>x</code> and <code>y</code>.
     */
    public double getArcLength()
    {
        double length = 0;
        TrcWaypoint waypoint = waypoints[0];

        for (int i = 1; i < waypoints.length; i++)
        {
            length += waypoint.distanceTo(waypoints[i]);
            waypoint = waypoints[i];
        }

        return length;
    }   //getArcLength

    /**
     * This method calculates the acceleration of each waypoint in the path.
     */
    public void inferAccelerations()
    {
        for (int i = 0; i < waypoints.length - 1; i++)
        {
            TrcWaypoint point = waypoints[i];
            TrcWaypoint next = waypoints[i + 1];
            double displacement = point.distanceTo(next);
            // Area of trapezoid: (v1+v2)/2 * t = d
            // t = d / ((v1+v2)/2)
            double t = displacement / TrcUtil.average(point.velocity, next.velocity);
            point.acceleration = (next.velocity - point.velocity) / t;
        }
        // Assume last waypoint has 0 acceleration
        waypoints[waypoints.length - 1].acceleration = 0;
    }   //inferAccelerations

    /**
     * Use velocity and position data to infer the timesteps of the waypoint.
     */
    public void inferTimeSteps()
    {
        for (int i = 0; i < waypoints.length - 1; i++)
        {
            TrcWaypoint point = waypoints[i];
            TrcWaypoint next = waypoints[i + 1];
            double displacement = point.distanceTo(next);
            // Area of trapezoid: (v1+v2)/2 * t = d
            // t = d / ((v1+v2)/2)
            point.timeStep = displacement / TrcUtil.average(point.velocity, next.velocity);
        }
        // Assume last waypoint has the same timestep as second to last
        waypoints[waypoints.length - 1].timeStep = waypoints[waypoints.length - 2].timeStep;
    }   //inferTimeSteps

}   //class TrcPath
