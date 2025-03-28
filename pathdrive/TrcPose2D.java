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

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

import trclib.dataprocessor.TrcUtil;

/**
 * This class implements a 2D pose object that represents the positional state of an object.
 */
public class TrcPose2D
{
    public final double x;
    public final double y;
    public final double angle;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x     specifies the x component of the position.
     * @param y     specifies the y component of the position.
     * @param angle specifies the angle.
     */
    public TrcPose2D(double x, double y, double angle)
    {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }   //TrcPose2D

    /**
     * Constructor: Create an instance of the object.
     *
     * @param data specifies an array with 3 elements: x, y and angle.
     * @throws ArrayIndexOutOfBoundsException if array size is less than 3.
     */
    public TrcPose2D(double[] data)
    {
        this(data[0], data[1], data[2]);
    }   //TrcPose2D

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x coordinate of the position.
     * @param y specifies the y coordinate of the position.
     */
    public TrcPose2D(double x, double y)
    {
        this(x, y, 0.0);
    }   //TrcPose2D

    /**
     * Constructor: Create an instance of the object.
     */
    public TrcPose2D()
    {
        this(0.0, 0.0, 0.0);
    }   //TrcPose2D

    /**
     * This method returns the string representation of the pose.
     *
     * @return string representation of the pose.
     */
    @Override
    public String toString()
    {
        return String.format(Locale.US, "(x=%f,y=%f,angle=%f)", x, y, angle);
    }   //toString

    /**
     * This method loads pose data from a CSV file either on the external file system or attached resources.
     *
     * @param path specifies the file system path or resource name.
     * @param loadFromResources specifies true if the data is from attached resources, false if from file system.
     * @return an array of poses.
     */
    public static TrcPose2D[] loadPosesFromCsv(String path, boolean loadFromResources)
    {
        TrcPose2D[] poses;

        if (!path.endsWith(".csv"))
        {
            throw new IllegalArgumentException(path + " is not a csv file!");
        }

        try
        {
            BufferedReader in = new BufferedReader(
                loadFromResources?
                    new InputStreamReader(
                        Objects.requireNonNull(TrcPose2D.class.getClassLoader()).getResourceAsStream(path)):
                    new FileReader(path));
            List<TrcPose2D> poseList = new ArrayList<>();
            String line;

            in.readLine();  // Get rid of the first header line
            while ((line = in.readLine()) != null)
            {
                String[] tokens = line.split(",");

                if (tokens.length != 3)
                {
                    throw new IllegalArgumentException("There must be 3 columns in the csv file!");
                }

                double[] elements = new double[tokens.length];

                for (int i = 0; i < elements.length; i++)
                {
                    elements[i] = Double.parseDouble(tokens[i]);
                }

                TrcPose2D pose = new TrcPose2D(elements[0], elements[1], elements[2]);
                poseList.add(pose);
            }
            in.close();

            poses = poseList.toArray(new TrcPose2D[0]);
        }
        catch (IOException e)
        {
            throw new RuntimeException(e);
        }

        return poses;
    }   //loadPosesFromCsv

    /**
     * This method returns the pose data in an array.
     *
     * @return pose data in an array.
     */
    public double[] getPoseData()
    {
        return new double[] {x, y, angle};
    }   //getPoseData

    /**
     * This method compares this pose with the specified pose for equality.
     *
     * @return true if equal, false otherwise.
     */
    @Override
    public boolean equals(Object o)
    {
        if (this == o)
            return true;
        if (o == null || getClass() != o.getClass())
            return false;
        TrcPose2D pose2D = (TrcPose2D) o;
        return Double.compare(pose2D.x, x) == 0 && Double.compare(pose2D.y, y) == 0 &&
               Double.compare(pose2D.angle, angle) == 0;
    }   //equals

    /**
     * This method returns the hash code of the values in this pose.
     *
     * @return pose hash code.
     */
    @Override
    public int hashCode()
    {
        return Objects.hash(x, y, angle);
    }   //hashCode

    /**
     * This method creates and returns a copy of this pose.
     *
     * @return a copy of this pose.
     */
    @Override
    public TrcPose2D clone()
    {
        return new TrcPose2D(this.x, this.y, this.angle);
    }   //clone

    /**
     * This method returns the vector form of this pose.
     *
     * @return vector form of this pose.
     */
    public RealVector toPosVector()
    {
        return TrcUtil.createVector(x, y);
    }   //toPoseVector

    /**
     * This method returns the distance of the specified pose to this pose.
     *
     * @param pose specifies the pose to calculate the distance to.
     *
     * @return distance to specified pose.
     */
    public double distanceTo(TrcPose2D pose)
    {
        return toPosVector().getDistance(pose.toPosVector());
    }   //distanceTo

    /**
     * This method returns a transformed pose relative to the given pose.
     *
     * @param pose           specifies the reference pose.
     * @param transformAngle specifies true to also transform angle, false to leave it alone.
     * @return pose relative to the given pose.
     */
    public TrcPose2D relativeTo(TrcPose2D pose, boolean transformAngle)
    {
        double deltaX = x - pose.x;
        double deltaY = y - pose.y;
        double newAngle = angle;

        RealVector newPos =
            TrcUtil.rotateCCW(MatrixUtils.createRealVector(new double[]{deltaX, deltaY}), pose.angle);
        if (transformAngle)
        {
            newAngle -= pose.angle;
        }

        return new TrcPose2D(newPos.getEntry(0), newPos.getEntry(1), newAngle);
    }   //relativeTo

    /**
     * This method returns a transformed pose relative to the given pose.
     *
     * @param pose specifies the reference pose.
     * @return pose relative to the given pose.
     */
    public TrcPose2D relativeTo(TrcPose2D pose)
    {
        return relativeTo(pose, true);
    }   //relativeTo

    /**
     * This method translates this pose with the x and y offset in reference to the angle of the pose.
     *
     * @param xOffset specifies the x offset in reference to the angle of the pose.
     * @param yOffset specifies the y offset in reference to the angle of the pose.
     * @return translated pose.
     */
    public TrcPose2D translatePose(double xOffset, double yOffset)
    {
        double angleRadians = Math.toRadians(angle);
        double cosAngle = Math.cos(angleRadians);
        double sinAngle = Math.sin(angleRadians);

        return new TrcPose2D(
            x + xOffset * cosAngle + yOffset * sinAngle,
            y - xOffset * sinAngle + yOffset * cosAngle,
            angle);
    }   //translatePose

    /**
     * This method rotates this pose with the specified angle.
     *
     * @param angle specifies the angle to rotate the pose.
     * @return rotated pose.
     */
    public TrcPose2D rotatePose(double angle)
    {
        RealVector vec = TrcUtil.rotateCW(toPosVector(), angle);
        return new TrcPose2D(vec.getEntry(0), vec.getEntry(1), this.angle + angle);
    }   //rotatePose

    /**
     * This method adds a relative pose to this pose and return the resulting pose. The relative pose has a
     * relative vector and relative angle from this pose.
     *
     * @param relativePose specifies the pose relative to the previous pose.
     * @return resulting pose.
     */
    public TrcPose2D addRelativePose(TrcPose2D relativePose)
    {
        RealVector vec =
            TrcUtil.createVector(this.x, this.y).add(TrcUtil.rotateCW(relativePose.toPosVector(), this.angle));
        return new TrcPose2D(vec.getEntry(0), vec.getEntry(1), this.angle + relativePose.angle);
    }   //addRelativePose

    /**
     * This method subtracts a relative pose from this pose and return the resulting pose. The relative pose has a
     * relative vector and relative angle from the resulting pose to this pose. In other words, adding the relative
     * pose to the resulting pose would yield this pose.
     *
     * @param relativePose specifies the relative pose from the resulting pose.
     * @return resulting pose.
     */
    public TrcPose2D subtractRelativePose(TrcPose2D relativePose)
    {
        RealVector vec =
            TrcUtil.createVector(this.x, this.y).subtract(TrcUtil.rotateCW(relativePose.toPosVector(), this.angle));
        return new TrcPose2D(vec.getEntry(0), vec.getEntry(1), this.angle - relativePose.angle);
    }   //subtractRelativePose

}   //class TrcPose2D
