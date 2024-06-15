/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib.path;

import java.util.ArrayList;

/**
 * This class builds a TrcPath for path following drive. The path can be built from two types of waypoints:
 * INCREMENTAL_PATH - points in the path are relative to their previous points.
 * REFERENCE_FRAME_PATH - points in the path are in the same reference frame as the starting pose.
 * <p>
 * For example, in traditional PID drive:
 * If the robot starts at the absolute field position of (x=48.0, y=12.0, heading=0.0) and we would want to
 * specify a path with 3 segments of relative movement:
 * 1. move forward 24 inches.
 * 2. turn left 90 degrees.
 * 3. strafe right 36 inches.
 * At the end, the robot will be at (x=48.0, y=72.0, heading=-90.0).
 * </p>
 * <p>
 * In INCREMENTAL_PATH mode, we would do:
 * TrcPathBuilder pathBuilder = new TrcPathBuilder(driveBase.getFieldPosition(), true)
 *                                  .append(new TrcPose2D(0.0, 24.0, -90.0))
 *                                  .append(new TrcPose2D(36.0, 0.0, 0.0));
 * </p>
 * In REFERENCE_FRAME_PATH mode, we would do:
 * TrcPathBuilder pathBuilder = new TrcPathBuilder(driveBase.getFieldPosition(), false)
 *                                  .append(driveBase.getFieldPosition())
 *                                  .append(new TrcPose2D(48.0, 36.0, -90.0))
 *                                  .append(new TrcPose2D(48.0, 72.0, -90.0));
 */
public class TrcPathBuilder
{
    private final ArrayList<TrcWaypoint> waypointList = new ArrayList<>();
    private final TrcPose2D startingPose;
    private final boolean incrementalPath;
    private final boolean inDegrees;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param inDegrees specifies true if appending points have headings with degree unit, false with radian units.
     */
    public TrcPathBuilder(TrcPose2D startingPose, boolean incrementalPath, boolean inDegrees)
    {
        this.startingPose = startingPose;
        this.incrementalPath = incrementalPath;
        this.inDegrees = inDegrees;
        waypointList.add(new TrcWaypoint(startingPose, null));
    }   //TrcPathBuilder

    /**
     * Constructor: Create an instance of the object.
     *
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     */
    public TrcPathBuilder(TrcPose2D startingPose, boolean incrementalPath)
    {
        this(startingPose, incrementalPath, true);
    }   //TrcPathBuilder

    /**
     * Appends the specified waypoint to the path.
     *
     * @param waypoint specifies the waypoint to be added to the path. If incrementalPoth is true, waypoint is
     *                 relative to the previous point, otherwise it is in the same reference frame as startingPose.
     *                 Note: the waypoint will be modified to be in the reference frame of startingPose.
     * @return this instance.
     */
    public TrcPathBuilder append(TrcWaypoint waypoint)
    {
        if (incrementalPath)
        {
            //
            // waypoint is relative to the previous point for an INCREMENTAL_PATH.
            // Transform it to be in the same reference frame as startingPose by adding it cumulatively to the
            // previous point.
            //
            TrcPose2D prevPose = waypointList.get(waypointList.size() - 1).pose;
            waypoint.pose.setAs(prevPose.addRelativePose(waypoint.pose));
        }
        waypointList.add(waypoint);

        return this;
    }   //append

    /**
     * Appends the specified pose and velocity to the path.
     *
     * @param pose specifies the pose to be added to the path. If incrementalPoth is true, pose is relative to the
     *             previous point, otherwise it is in the same reference frame as startingPose.
     * @param velocity specifies the velocity at the pose.
     * @return this instance.
     */
    public TrcPathBuilder append(TrcPose2D pose, TrcPose2D velocity)
    {
        return append(new TrcWaypoint(pose, velocity));
    }   //append

    /**
     * Appends the specified pose to the path.
     *
     * @param pose specifies the pose to be added to the path. If incrementalPoth is true, pose is relative to the
     *             previous point, otherwise it is in the same reference frame as startingPose.
     * @return this instance.
     */
    public TrcPathBuilder append(TrcPose2D pose)
    {
        return append(new TrcWaypoint(pose, null));
    }   //append

    /**
     * This method returns the TrcPath built. All waypoints in the path are in the same reference frame as
     * startingPose.
     * Note: waypoint headings are relative to the field, regardless of reference frame.
     *
     * @return resulting TrcPath.
     */
    public TrcPath toPath()
    {
        return new TrcPath(inDegrees, waypointList.toArray(new TrcWaypoint[0]));
    }   //toPath

    /**
     * This method returns the TrcPath built. All waypoints in the path are relative to startingPose.
     * Note: waypoint headings are relative to the field, regardless of reference frame.
     *
     * @return resulting TrcPath relative to startingPose.
     */
    public TrcPath toRelativeStartPath()
    {
        return toPath().relativeTo(startingPose);
    }   //toRelativeStartPath

}   //class TrcPathBuilder
