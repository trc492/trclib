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
 * This class implements a 2D pose object that represents the positional and orientation state of a robot
 * or a coordinate frame on a flat playing field. It follows the TrcLib convention: X positive is Right,
 * Y positive is Forward, and angles increase Clockwise (CW) Positive.
 */
public class TrcPose2D
{
    public double x;
    public double y;
    public double angle;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x component of the position.
     * @param y specifies the y component of the position.
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
     * This method compares this pose with the specified pose for equality.
     *
     * @return true if equal, false otherwise.
     */
    @Override
    public boolean equals(Object o)
    {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        TrcPose2D pose2D = (TrcPose2D) o;
        return Double.compare(pose2D.x, x) == 0 &&
               Double.compare(pose2D.y, y) == 0 &&
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
     * This method sets this pose to be the same as the given pose.
     *
     * @param pose specifies the pose to make this pose equal to.
     */
    public void setAs(TrcPose2D pose)
    {
        this.x = pose.x;
        this.y = pose.y;
        this.angle = pose.angle;
    }   //setAs

    /**
     * This method returns the vector form of this pose's positional components.
     *
     * @return vector form of this pose.
     */
    public RealVector toPosVector()
    {
        return MatrixUtils.createRealVector(new double[]{x, y});
    }   //toPosVector

    /**
     * This method returns the linear distance of the specified pose to this pose.
     *
     * @param pose specifies the target pose to calculate the distance to.
     * @return linear distance to the specified pose.
     */
    public double distanceTo(TrcPose2D pose)
    {
        return toPosVector().getDistance(pose.toPosVector());
    }   //distanceTo

    //
    // Linear Vector Arithmetic Methods (Parallel to TrcPose3D)
    //

    /**
     * This method adds another pose to this one (Vector addition).
     * Positions compound lineally, orientations remain untouched.
     * Parallel to TrcPose3D.add.
     *
     * @param pose specifies the pose to be added.
     * @return resulting summed pose.
     */
    public TrcPose2D add(TrcPose2D pose)
    {
        return new TrcPose2D(this.x + pose.x, this.y + pose.y, this.angle);
    }   //add

    /**
     * This method subtracts another pose from this one (Vector subtraction).
     * Parallel to TrcPose3D.subtract.
     *
     * @param pose specifies the pose to be subtracted.
     * @return resulting difference pose.
     */
    public TrcPose2D subtract(TrcPose2D pose)
    {
        return new TrcPose2D(this.x - pose.x, this.y - pose.y, this.angle);
    }   //subtract

    /**
     * This method negates the positional translations of this pose.
     * Parallel to TrcPose3D.negate.
     *
     * @return negated translation pose.
     */
    public TrcPose2D negate()
    {
        return new TrcPose2D(-this.x, -this.y, this.angle);
    }   //negate

    /**
     * This method scales the positional translations of this pose by a constant multiplier.
     * Parallel to TrcPose3D.scale.
     *
     * @param scale specifies the scalar multiplier.
     * @return scaled pose.
     */
    public TrcPose2D scale(double scale)
    {
        return new TrcPose2D(this.x * scale, this.y * scale, this.angle);
    }   //scale

    //
    // Rigid Body Coordinate Frame Transformations
    //

    /**
     * This method translates this pose with the x and y offset in reference to its own local angle.
     * Parallel to TrcPose3D.translatePose.
     *
     * @param xOffset specifies the x offset in reference to the angle of the pose.
     * @param yOffset specifies the y offset in reference to the angle of the pose.
     * @return translated pose.
     */
    public TrcPose2D translatePose(double xOffset, double yOffset)
    {
        return this.addRelativePose(new TrcPose2D(xOffset, yOffset, 0.0));
    }   //translatePose

    /**
     * This method rotates this pose's positional coordinate vector around the world origin center point.
     * Maps perfectly to TrcLib CW-positive convention.
     *
     * @param angle specifies the angle in degrees to rotate the coordinates by.
     * @return rotated pose.
     */
    public TrcPose2D rotatePose(double angle)
    {
        RealVector vec = TrcUtil.rotateCW(toPosVector(), angle);
        return new TrcPose2D(vec.getEntry(0), vec.getEntry(1), this.angle + angle);
    }   //rotatePose

    /**
     * This method adds a relative pose transformation onto this tracking base frame.
     * The relative pose contains a local vector translation and relative orientation offset.
     * Parallel to TrcPose3D.addRelativePose.
     *
     * @param relativePose specifies the local child pose relative to this parent frame.
     * @return consolidated global field pose.
     */
    public TrcPose2D addRelativePose(TrcPose2D relativePose)
    {
        RealVector baseVec = MatrixUtils.createRealVector(new double[]{this.x, this.y});
        RealVector rotatedRelativeVec = TrcUtil.rotateCW(relativePose.toPosVector(), this.angle);
        RealVector combinedVec = baseVec.add(rotatedRelativeVec);

        return new TrcPose2D(combinedVec.getEntry(0), combinedVec.getEntry(1), this.angle + relativePose.angle);
    }   //addRelativePose

    /**
     * This method returns a transformed pose relative to the given reference base pose.
     * Maps perfectly to TrcLib Left-Handed CW-positive un-rotation transformations.
     * Unified replacement for relativeFrom/subtractRelativePose.
     *
     * @param pose           specifies the reference base pose.
     * @param transformAngle specifies true to also transform angle, false to leave it alone.
     * @return pose relative to the given pose.
     */
    public TrcPose2D relativeTo(TrcPose2D pose, boolean transformAngle)
    {
        double deltaX = this.x - pose.x;
        double deltaY = this.y - pose.y;
        double newAngle = this.angle;
        RealVector deltaVec = MatrixUtils.createRealVector(new double[]{deltaX, deltaY});
        // Un-rotate the delta vector into the target pose's frame by flipping direction
        RealVector newPos = TrcUtil.rotateCW(deltaVec, -pose.angle);
        if (transformAngle)
        {
            newAngle -= pose.angle;
        }
        return new TrcPose2D(newPos.getEntry(0), newPos.getEntry(1), newAngle);
    }   //relativeTo

    /**
     * This method returns a transformed pose relative to the given reference base pose.
     * Compounds the orientation angles by default.
     *
     * @param pose specifies the reference base pose.
     * @return pose relative to the given pose.
     */
    public TrcPose2D relativeTo(TrcPose2D pose)
    {
        return relativeTo(pose, true);
    }   //relativeTo

    /**
     * This method calculates the mathematical transformation inverse of this pose.
     * Formulated for rigid-body CW positive coordinate frame compliance.
     *
     * @return inverted coordinate frame pose mapping.
     */
    public TrcPose2D inverse()
    {
        double invAngle = -this.angle;
        RealVector negatedPos = this.toPosVector().mapMultiply(-1.0);
        RealVector invPos = TrcUtil.rotateCW(negatedPos, invAngle);
        return new TrcPose2D(invPos.getEntry(0), invPos.getEntry(1), invAngle);
    }   //inverse

}   //class TrcPose2D
