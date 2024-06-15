/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

import org.apache.commons.math3.linear.RealVector;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

import trclib.robotcore.TrcUtil;

/**
 * This class implements a 3D pose object that represents the positional state of an object.
 */
public class TrcPose3D
{
    public double x;
    public double y;
    public double z;
    public double yaw;
    public double pitch;
    public double roll;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x component of the pose.
     * @param y specifies the y component of the pose.
     * @param z specifies the z component of the pose.
     * @param yaw specifies the yaw angle.
     * @param pitch specifies the pitch angle.
     * @param roll specifies the roll angle.
     */
    public TrcPose3D(double x, double y, double z, double yaw, double pitch, double roll)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }   //TrcPose3D

    /**
     * Constructor: Create an instance of the object.
     *
     * @param data specifies an array with 6 elements: x, y, z, yaw, pitch and roll.
     */
    public TrcPose3D(double[] data)
    {
        this(data[0], data[1], data[2], data[3], data[4], data[5]);
    }   //TrcPose3D

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x coordinate of the pose.
     * @param y specifies the y coordinate of the pose.
     * @param z specifies the z coordinate of the pose.
     */
    public TrcPose3D(double x, double y, double z)
    {
        this(x, y, z, 0.0, 0.0, 0.0);
    }   //TrcPose3D

    /**
     * Constructor: Create an instance of the object.
     */
    public TrcPose3D()
    {
        this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }   //TrcPose3D

    /**
     * This method returns the string representation of the pose.
     *
     * @return string representation of the pose.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "(x=%.1f,y=%.1f,z=%.1f,yaw=%.1f,pitch=%.1f,roll=%.1f)", x, y, z, yaw, pitch, roll);
    }   //toString

    /**
     * This method loads pose data from a CSV file either on the external file system or attached resources.
     *
     * @param path specifies the file system path or resource name.
     * @param loadFromResources specifies true if the data is from attached resources, false if from file system.
     * @return an array of poses.
     */
    public static TrcPose3D[] loadPosesFromCsv(String path, boolean loadFromResources)
    {
        TrcPose3D[] poses;

        if (!path.endsWith(".csv"))
        {
            throw new IllegalArgumentException(path + " is not a csv file!");
        }

        try
        {
            BufferedReader in = new BufferedReader(
                loadFromResources?
                    new InputStreamReader(TrcPose3D.class.getClassLoader().getResourceAsStream(path)):
                    new FileReader(path));
            List<TrcPose3D> poseList = new ArrayList<>();
            String line;

            in.readLine();  // Get rid of the first header line
            while ((line = in.readLine()) != null)
            {
                String[] tokens = line.split(",");

                if (tokens.length != 6)
                {
                    throw new IllegalArgumentException("There must be 6 columns in the csv file!");
                }

                double[] elements = new double[tokens.length];

                for (int i = 0; i < elements.length; i++)
                {
                    elements[i] = Double.parseDouble(tokens[i]);
                }

                TrcPose3D pose = new TrcPose3D(
                    elements[0], elements[1], elements[2], elements[3], elements[4], elements[5]);
                poseList.add(pose);
            }
            in.close();

            poses = poseList.toArray(new TrcPose3D[0]);
        }
        catch (IOException e)
        {
            throw new RuntimeException(e);
        }

        return poses;
    }   //loadPosesFromCsv

    /**
     * This method compares this pose with the specified pose for equality.
     *
     * @return true if equal, false otherwise.
     */
    @Override
    public boolean equals(Object o)
    {
        boolean equality;

        if (this == o)
        {
            equality = true;
        }
        else if (o == null || getClass() != o.getClass())
        {
            equality = false;
        }
        else
        {
            TrcPose3D pose = (TrcPose3D) o;
            equality = Double.compare(pose.x, x) == 0 &&
                       Double.compare(pose.y, y) == 0 &&
                       Double.compare(pose.z, z) == 0 &&
                       Double.compare(pose.yaw, yaw) == 0 &&
                       Double.compare(pose.pitch, pitch) == 0 &&
                       Double.compare(pose.roll, roll) == 0;
        }

        return equality;
    }   //equals

    /**
     * This method returns the hash code of the values in this pose.
     *
     * @return pose hash code.
     */
    @Override
    public int hashCode()
    {
        return Objects.hash(x, y, z, yaw, pitch, roll);
    }   //hashCode

    /**
     * This method creates and returns a copy of this pose.
     *
     * @return a copy of this pose.
     */
    public TrcPose3D clone()
    {
        return new TrcPose3D(this.x, this.y, this.z, this.yaw, this.pitch, this.roll);
    }   //clone

    /**
     * This method converts the Pose3D to a Pose2D.
     *
     * @return converted Pose2D.
     */
    public TrcPose2D toPose2D()
    {
        // TrcPose2D has a positive clockwise angle but TrcPose3D has a positive counter clockwise yaw.
        return new TrcPose2D(x, y, -yaw);
    }   //toPose2D

    /**
     * This method returns the vector form of this pose.
     *
     * @return vector form of this pose.
     */
    public RealVector toPosVector()
    {
        return TrcUtil.createVector(x, y, z);
    }   //toPoseVector

    /**
     * This method returns the distance of the specified pose to this pose.
     *
     * @param pose specifies the pose to calculate the distance to.
     *
     * @return distance to specified pose.
     */
    public double distanceTo(TrcPose3D pose)
    {
        return toPosVector().getDistance(pose.toPosVector());
    }   //distanceTo

    /**
     * This method sets this pose to be the same as the given pose.
     *
     * @param pose specifies the pose to make this pose equal to.
     */
    public void setAs(TrcPose3D pose)
    {
        this.x = pose.x;
        this.y = pose.y;
        this.z = pose.z;
        this.yaw = pose.yaw;
        this.pitch = pose.pitch;
        this.roll = pose.roll;
    }   //setAs

    // TODO: implement these methods.
//    /**
//     * This method returns a transformed pose relative to the given pose.
//     *
//     * @param pose           specifies the reference pose.
//     * @param transformAngle specifies true to also transform angle, false to leave it alone.
//     * @return pose relative to the given pose.
//     */
//    public TrcPose3D relativeTo(TrcPose3D pose, boolean transformAngle)
//    {
//        double deltaX = x - pose.x;
//        double deltaY = y - pose.y;
//        double deltaZ = z - pose.z;
//        double newAngle = angle;
//
//        RealVector newPos =
//            TrcUtil.rotateCCW(MatrixUtils.createRealVector(new double[]{deltaX, deltaY}), pose.angle);
//        if (transformAngle)
//        {
//            newAngle -= pose.angle;
//        }
//
//        return new TrcPose3D(newPos.getEntry(0), newPos.getEntry(1), newAngle);
//    }   //relativeTo
//
//    /**
//     * This method returns a transformed pose relative to the given pose.
//     *
//     * @param pose specifies the reference pose.
//     * @return pose relative to the given pose.
//     */
//    public TrcPose3D relativeTo(TrcPose3D pose)
//    {
//        return relativeTo(pose, true);
//    }   //relativeTo
//
//    /**
//     * This method translates this pose with the x and y offset in reference to the angle of the pose.
//     *
//     * @param xOffset specifies the x offset in reference to the angle of the pose.
//     * @param yOffset specifies the y offset in reference to the angle of the pose.
//     * @param zOffset specifies the z offset in reference to the angle of the pose.
//     * @return translated pose.
//     */
//    public TrcPose3D translatePose(double xOffset, double yOffset, double zOffset)
//    {
//        TrcPose3D newPose = clone();
//        double angleRadians = Math.toRadians(newPose.angle);
//        double cosAngle = Math.cos(angleRadians);
//        double sinAngle = Math.sin(angleRadians);
//
//        newPose.x += xOffset * cosAngle + yOffset * sinAngle;
//        newPose.y += -xOffset * sinAngle + yOffset * cosAngle;
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceInfo(moduleName, "xOffset=%.1f, yOffset=%.1f, Pose:%s, newPose:%s",
//                xOffset, yOffset, this, newPose);
//        }
//
//        return newPose;
//    }   //translatePose
//
//    /**
//     * This method adds a relative pose to the this pose and return the resulting pose. The relative pose has a
//     * relative vector and relative angle from this pose.
//     *
//     * @param relativePose specifies the pose relative to the previous pose.
//     * @return resulting pose.
//     */
//    public TrcPose3D addRelativePose(TrcPose3D relativePose)
//    {
//        RealVector vec = TrcUtil.createVector(this.x, this.y).add(
//                TrcUtil.rotateCW(relativePose.toPosVector(), this.angle));
//        return new TrcPose3D(vec.getEntry(0), vec.getEntry(1), this.angle + relativePose.angle);
//    }   //addRelativePose

}   //class TrcPose3D
