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

import org.apache.commons.math3.linear.RealVector;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
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
     * This method opens a CSV reader either from a file or from attached resources.
     *
     * @param path specifies the file system path or resource name.
     * @param loadFromResources specifies true if the data is from attached resources, false if from file system.
     * @return created reader.
     * @throws IOException when no resource found.
     */
    private static Reader openCsvReader(String path, boolean loadFromResources)
        throws IOException
    {
        if (loadFromResources)
        {
            InputStream inputStream =
                Objects.requireNonNull(TrcPose2D.class.getClassLoader()).getResourceAsStream(path);

            if (inputStream == null)
            {
                throw new IOException("Resource not found: " + path);
            }

            return new InputStreamReader(inputStream);
        }

        return new FileReader(path);
    }   //openCsvReader

    /**
     * This method loads pose data from a CSV file either on the external file system or attached resources.
     *
     * @param path specifies the file system path or resource name.
     * @param loadFromResources specifies true if the data is from attached resources, false if from file system.
     * @return an array of poses.
     */
    public static TrcPose2D[] loadPosesFromCsv(String path, boolean loadFromResources)
    {
        if (!path.toLowerCase(Locale.ROOT).endsWith(".csv"))
        {
            throw new IllegalArgumentException(path + " is not a csv file!");
        }

        try (BufferedReader reader = new BufferedReader(openCsvReader(path, loadFromResources)))
        {
            List<TrcPose2D> poseList = new ArrayList<>();

            reader.readLine();  // Get rid of the first header line.

            String line;
            while ((line = reader.readLine()) != null)
            {
                if (line.isEmpty())
                {
                    continue;
                }

                String[] tokens = line.split(",");

                if (tokens.length != 3)
                {
                    throw new IllegalArgumentException("There must be 3 columns in the csv file!");
                }

                double x = Double.parseDouble(tokens[0].trim());
                double y = Double.parseDouble(tokens[1].trim());
                double angle = Double.parseDouble(tokens[2].trim());

                poseList.add(new TrcPose2D(x, y, angle));
            }

            return poseList.toArray(new TrcPose2D[0]);
        }
        catch (IOException e)
        {
            throw new RuntimeException(e);
        }
    }   //loadPosesFromCsv

    /**
     * This method compares this pose with the specified pose for equality.
     *
     * @param o specifies the object to compare with this pose.
     * @return true if equal, false otherwise.
     */
    @Override
    public boolean equals(Object o)
    {
        if (this == o)
        {
            return true;
        }

        if (o == null || getClass() != o.getClass())
        {
            return false;
        }

        TrcPose2D pose = (TrcPose2D) o;

        return Double.compare(pose.x, x) == 0 &&
               Double.compare(pose.y, y) == 0 &&
               Double.compare(pose.angle, angle) == 0;
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
        return TrcUtil.createVector(x, y);
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
    // Linear Vector Arithmetic Methods.
    //

    /**
     * This method adds another pose to this one (vector addition).
     * Positions are added linearly and this pose's orientation is preserved.
     *
     * @param pose specifies the pose to be added.
     * @return resulting summed pose.
     */
    public TrcPose2D add(TrcPose2D pose)
    {
        return new TrcPose2D(this.x + pose.x, this.y + pose.y, this.angle);
    }   //add

    /**
     * This method subtracts another pose from this one (vector subtraction).
     * This pose's orientation is preserved.
     *
     * @param pose specifies the pose to be subtracted.
     * @return resulting difference pose.
     */
    public TrcPose2D subtract(TrcPose2D pose)
    {
        return new TrcPose2D(this.x - pose.x, this.y - pose.y, this.angle);
    }   //subtract

    /**
     * This method negates the positional translation of this pose.
     * The orientation is preserved.
     *
     * @return pose with negated translation.
     */
    public TrcPose2D negate()
    {
        return new TrcPose2D(-this.x, -this.y, this.angle);
    }   //negate

    /**
     * This method scales the positional translation of this pose by a constant multiplier.
     * The orientation is preserved.
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
     * This method translates this pose by the specified offset in its local coordinate frame.
     *
     * @param xOffset specifies the x offset in the local coordinate frame.
     * @param yOffset specifies the y offset in the local coordinate frame.
     * @return translated pose.
     */
    public TrcPose2D translatePose(double xOffset, double yOffset)
    {
        RealVector offset = TrcUtil.createVector(xOffset, yOffset);
        RealVector rotatedOffset = TrcUtil.rotateCW(offset, angle);
        return new TrcPose2D(x + rotatedOffset.getEntry(0), y + rotatedOffset.getEntry(1), angle);
    }   //translatePose

    /**
     * This method rotates this pose's positional coordinate around the world origin.
     * The pose's orientation is unchanged.
     *
     * @param rotationAngle specifies the rotation angle in degrees.
     * @return a new pose with the rotated positional coordinate and unchanged orientation.
     */
    public TrcPose2D rotate(double rotationAngle)
    {
        RealVector rotatedPos = TrcUtil.rotateCW(toPosVector(), rotationAngle);
        return new TrcPose2D(rotatedPos.getEntry(0), rotatedPos.getEntry(1), angle);
    }   //rotate

    /**
     * This method rotates this pose around the world origin.
     * Both the positional coordinate and the pose's orientation are rotated by
     * the same amount.
     *
     * @param rotationAngle specifies the rotation angle in degrees.
     * @return a new pose with the rotated positional coordinate and orientation.
     */
    public TrcPose2D rotatePose(double rotationAngle)
    {
        RealVector rotatedPos = TrcUtil.rotateCW(toPosVector(), rotationAngle);
        return new TrcPose2D(rotatedPos.getEntry(0), rotatedPos.getEntry(1), angle + rotationAngle);
    }   //roatePose

    /**
     * This method adds a relative pose transformation to this pose.
     * The relative pose's translation is expressed in this pose's local coordinate frame.
     *
     * @param relativePose specifies the relative pose to add.
     * @return resulting global pose.
     */
    public TrcPose2D addRelativePose(TrcPose2D relativePose)
    {
        RealVector rotatedRelativePos = TrcUtil.rotateCW(relativePose.toPosVector(), angle);
        return new TrcPose2D(
            x + rotatedRelativePos.getEntry(0), y + rotatedRelativePos.getEntry(1), angle + relativePose.angle);
    }   //addRelativePose

    /**
     * This method returns this pose expressed relative to the specified reference pose.
     *
     * @param pose specifies the reference pose.
     * @param transformAngle specifies true to express the angle relative to the reference pose,
     *                       false to preserve this pose's absolute angle.
     * @return this pose expressed in the reference pose's coordinate frame.
     */
    public TrcPose2D relativeTo(TrcPose2D pose, boolean transformAngle)
    {
        RealVector delta = TrcUtil.createVector(this.x - pose.x, this.y - pose.y);
        RealVector relativePos = TrcUtil.rotateCW(delta, -pose.angle);
        double relativeAngle = transformAngle? this.angle - pose.angle: this.angle;
        return new TrcPose2D(relativePos.getEntry(0), relativePos.getEntry(1), relativeAngle);
    }   //relativeTo

    /**
     * This method returns this pose expressed relative to the specified reference pose,
     * including transformation of its orientation.
     *
     * @param pose specifies the reference pose.
     * @return this pose expressed in the reference pose's coordinate frame.
     */
    public TrcPose2D relativeTo(TrcPose2D pose)
    {
        return relativeTo(pose, true);
    }   //relativeTo

    /**
     * This method returns the inverse of this pose transformation.
     *
     * @return inverse pose transformation.
     */
    public TrcPose2D inverse()
    {
        double inverseAngle = -angle;
        RealVector inversePos = TrcUtil.rotateCW(toPosVector().mapMultiply(-1.0), inverseAngle);
        return new TrcPose2D(inversePos.getEntry(0), inversePos.getEntry(1), inverseAngle);
    }   //inverse

}   //class TrcPose2D
