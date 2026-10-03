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

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
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
 * This class implements a 3D pose object that represents the positional and orientation state of an object.
 *
 * <p>The TRC coordinate convention is:
 * <ul>
 * <li>X rotation: pitch</li>
 * <li>Y rotation: roll</li>
 * <li>Z rotation: yaw</li>
 * <li>Yaw is clockwise-positive</li>
 * </ul>
 */
public class TrcPose3D
{
    public double x;
    public double y;
    public double z;
    public double pitch; // Rotation around X-axis.
    public double roll;  // Rotation around Y-axis.
    public double yaw;   // Rotation around Z-axis, clockwise-positive.

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x component of the pose.
     * @param y specifies the y component of the pose.
     * @param z specifies the z component of the pose.
     * @param pitch specifies the pitch angle (rotation about the X axis).
     * @param roll specifies the roll angle (rotation about the Y axis).
     * @param yaw specifies the yaw angle (rotation about the Z axis).
     */
    public TrcPose3D(double x, double y, double z, double pitch, double roll, double yaw)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        this.pitch = pitch;
        this.roll = roll;
        this.yaw = yaw;
    }   //TrcPose3D

    /**
     * Constructor: Create an instance of the object.
     *
     * @param data specifies an array with 6 elements: x, y, z, pitch, roll, and yaw.
     */
    public TrcPose3D(double[] data)
    {
        this(data[0], data[1], data[2], data[3], data[4], data[5]);
    }   //TrcPose3D

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x component of the pose.
     * @param y specifies the y component of the pose.
     * @param z specifies the z component of the pose.
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

    @Override
    public String toString()
    {
        return "(x=" + x + ",y=" + y + ",z=" + z +
               ",pitch=" + pitch + ",roll=" + roll + ",yaw=" + yaw + ")";
    }   //toString

    /**
     * This method creates TRC 3D poses from a CSV file.
     *
     * @param path specifies resource stream name or CSV file path.
     * @param loadFromResources specifies true if path is a resource stream name,
     *                          false if it is a CSV file path.
     * @return array of TrcPose3D poses.
     */
    public static TrcPose3D[] loadPosesFromCsv(String path, boolean loadFromResources)
    {
        if (!path.toLowerCase(Locale.ROOT).endsWith(".csv"))
        {
            throw new IllegalArgumentException(path + " is not a csv file!");
        }

        try (BufferedReader reader = new BufferedReader(openCsvReader(path, loadFromResources)))
        {
            List<TrcPose3D> poseList = new ArrayList<>();

            // Skip the header.
            reader.readLine();

            String line;
            while ((line = reader.readLine()) != null)
            {
                if (line.isEmpty())
                {
                    continue;
                }

                String[] tokens = line.split(",");

                if (tokens.length != 6)
                {
                    throw new IllegalArgumentException("There must be 6 columns in the csv file!");
                }

                double x = Double.parseDouble(tokens[0].trim());
                double y = Double.parseDouble(tokens[1].trim());
                double z = Double.parseDouble(tokens[2].trim());
                double pitch = Double.parseDouble(tokens[3].trim());
                double roll = Double.parseDouble(tokens[4].trim());
                double yaw = Double.parseDouble(tokens[5].trim());

                poseList.add(new TrcPose3D(x, y, z, pitch, roll, yaw));
            }

            return poseList.toArray(new TrcPose3D[0]);
        }
        catch (IOException e)
        {
            throw new RuntimeException(e);
        }
    }   //loadPosesFromCsv

    /**
     * Opens a CSV reader from either a file or an attached resource.
     */
    private static Reader openCsvReader(String path, boolean loadFromResources)
        throws IOException
    {
        if (loadFromResources)
        {
            InputStream inputStream =
                Objects.requireNonNull(TrcPose3D.class.getClassLoader()).getResourceAsStream(path);

            if (inputStream == null)
            {
                throw new IOException("Resource not found: " + path);
            }

            return new InputStreamReader(inputStream);
        }

        return new FileReader(path);
    }   //openCsvReader

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

        TrcPose3D pose = (TrcPose3D) o;

        return Double.compare(pose.x, x) == 0 &&
               Double.compare(pose.y, y) == 0 &&
               Double.compare(pose.z, z) == 0 &&
               Double.compare(pose.pitch, pitch) == 0 &&
               Double.compare(pose.roll, roll) == 0 &&
               Double.compare(pose.yaw, yaw) == 0;
    }   //equals

    /**
     * This method returns the hash code of the values in this pose.
     *
     * @return pose hash code.
     */
    @Override
    public int hashCode()
    {
        return Objects.hash(x, y, z, pitch, roll, yaw);
    }   //hashCode

    /**
     * This method creates and returns a copy of this pose.
     *
     * @return a copy of this pose.
     */
    @Override
    public TrcPose3D clone()
    {
        return new TrcPose3D(this.x, this.y, this.z, this.pitch, this.roll, this.yaw);
    }   //clone

    /**
     * This method sets this pose to be the same as the given pose.
     *
     * @param pose specifies the pose to copy.
     */
    public void setAs(TrcPose3D pose)
    {
        this.x = pose.x;
        this.y = pose.y;
        this.z = pose.z;
        this.pitch = pose.pitch;
        this.roll = pose.roll;
        this.yaw = pose.yaw;
    }   //setAs

    /**
     * This method converts the pose to a TrcPose2D.
     *
     * @return converted TrcPose2D.
     */
    public TrcPose2D toTrcPose2D()
    {
        return new TrcPose2D(x, y, yaw);
    }   //toTrcPose2D

    /**
     * This method returns the positional vector of this pose.
     *
     * @return positional vector.
     */
    public RealVector toPosVector()
    {
        return TrcUtil.createVector(x, y, z);
    }   //toPosVector

    /**
     * This method returns the distance to the specified pose.
     *
     * @param pose specifies the pose to calculate the distance to.
     * @return distance to specified pose.
     */
    public double distanceTo(TrcPose3D pose)
    {
        return toPosVector().getDistance(pose.toPosVector());
    }   //distanceTo

    //
    // Linear Vector Arithmetic Methods.
    //

    /**
     * Performs component-wise positional addition. The orientation of this pose
     * is retained.
     *
     * @param pose specifies the pose to add.
     * @return resulting pose.
     */
    public TrcPose3D add(TrcPose3D pose)
    {
        return new TrcPose3D(this.x + pose.x, this.y + pose.y, this.z + pose.z, this.pitch, this.roll, this.yaw);
    }   //add

    /**
     * Performs component-wise positional subtraction. The orientation of this pose
     * is retained.
     *
     * @param pose specifies the pose to subtract.
     * @return resulting pose.
     */
    public TrcPose3D subtract(TrcPose3D pose)
    {
        return new TrcPose3D(this.x - pose.x, this.y - pose.y, this.z - pose.z, this.pitch, this.roll, this.yaw);
    }   //subtract

    /**
     * Negates the positional components of this pose. The orientation is retained.
     *
     * @return resulting pose.
     */
    public TrcPose3D negate()
    {
        return new TrcPose3D(-this.x, -this.y, -this.z, this.pitch, this.roll, this.yaw);
    }   //negate

    /**
     * Scales the positional components of this pose. The orientation is retained.
     *
     * @param factor specifies the scale factor.
     * @return resulting pose.
     */
    public TrcPose3D scale(double factor)
    {
        return new TrcPose3D(this.x * factor, this.y * factor, this.z * factor, this.pitch, this.roll, this.yaw);
    }   //scale

    //
    // Rigid Body Coordinate Frame Transformations
    //

    /**
     * Translates this pose by an offset expressed in its local coordinate frame.
     *
     * @param xOffset specifies the local X offset.
     * @param yOffset specifies the local Y offset.
     * @param zOffset specifies the local Z offset.
     * @return resulting pose.
     */
    public TrcPose3D translatePose(double xOffset, double yOffset, double zOffset)
    {
        TrcPose3D offsetPose = new TrcPose3D(xOffset, yOffset, zOffset);
        TrcPose3D rotatedOffset = offsetPose.rotate(this.pitch, this.roll, this.yaw);

        return add(rotatedOffset);
    }   //translatePose

    /**
     * This method rotates this pose's positional coordinate around the world origin.
     * The pose's orientation is unchanged.
     *
     * @param pitch specifies the rotation angle around the X-axis in degrees.
     * @param roll specifies the rotation angle around the Y-axis in degrees.
     * @param yaw specifies the clockwise rotation angle around the Z-axis in degrees.
     * @return a new pose with the rotated positional coordinate and unchanged orientation.
     */
    public TrcPose3D rotate(double pitch, double roll, double yaw)
    {
        Rotation rotation = getRotation(pitch, roll, yaw);
        Vector3D position = new Vector3D(x, y, z);
        Vector3D rotatedPosition = rotation.applyTo(position);

        return new TrcPose3D(
            rotatedPosition.getX(), rotatedPosition.getY(), rotatedPosition.getZ(), this.pitch, this.roll, this.yaw);
    }   //rotate

    /**
     * This method rotates this pose around the world origin. Both the positional
     * coordinate and the pose's orientation are rotated by the same amount.
     *
     * @param pitch specifies the rotation angle around the X-axis in degrees.
     * @param roll specifies the rotation angle around the Y-axis in degrees.
     * @param yaw specifies the clockwise rotation angle around the Z-axis in degrees.
     * @return a new pose with the rotated positional coordinate and orientation.
     */
    public TrcPose3D rotatePose(double pitch, double roll, double yaw)
    {
        Rotation rotation = getRotation(pitch, roll, yaw);
        Vector3D position = new Vector3D(x, y, z);
        Vector3D rotatedPosition = rotation.applyTo(position);
        TrcPose3D rotatedPose = new TrcPose3D(
            rotatedPosition.getX(), rotatedPosition.getY(), rotatedPosition.getZ());

        rotatedPose.setOrientation(rotation.applyTo(getRotation()));

        return rotatedPose;
    }   //rotatePose

    /**
     * Adds a pose expressed in this pose's local coordinate frame and returns
     * the resulting global pose.
     *
     * @param relativePose specifies the relative pose.
     * @return resulting global pose.
     */
    public TrcPose3D addRelativePose(TrcPose3D relativePose)
    {
        TrcPose3D rotatedRelativePose = relativePose.rotate(this.pitch, this.roll, this.yaw);
        TrcPose3D finalPose = add(rotatedRelativePose);
        Rotation currentRotation = getRotation();
        Rotation relativeRotation = relativePose.getRotation();
        /*
         * Apache Commons Math applies the argument rotation after the
         * instance rotation. Therefore:
         *
         *     relativeRotation.applyTo(currentRotation)
         *
         * represents:
         *
         *     currentRotation * relativeRotation
         */
        Rotation compoundedRotation = relativeRotation.applyTo(currentRotation);
        finalPose.setOrientation(compoundedRotation);

        return finalPose;
    }   //addRelativePose

    /**
     * Returns this pose expressed in the coordinate frame of the specified
     * reference pose.
     *
     * @param pose specifies the reference pose.
     * @param transformAngle specifies whether orientation should also be transformed.
     * @return this pose expressed relative to the reference pose.
     */
    public TrcPose3D relativeTo(TrcPose3D pose, boolean transformAngle)
    {
        double deltaX = x - pose.x;
        double deltaY = y - pose.y;
        double deltaZ = z - pose.z;
        Rotation referenceInverse = pose.getRotation().revert();
        Vector3D delta = new Vector3D(deltaX, deltaY, deltaZ);
        Vector3D local = referenceInverse.applyTo(delta);
        TrcPose3D relativePose = new TrcPose3D(local.getX(), local.getY(), local.getZ());

        if (transformAngle)
        {
            /*
             * Desired rotation:
             *
             *     R_relative = R_reference^-1 * R_this
             *
             * With Apache Rotation.applyTo(), this is obtained by applying
             * the reference inverse first in the composition:
             */
            Rotation relativeRotation = getRotation().applyTo(referenceInverse);
            relativePose.setOrientation(relativeRotation);
        }
        else
        {
            relativePose.pitch = pitch;
            relativePose.roll = roll;
            relativePose.yaw = yaw;
        }

        return relativePose;
    }   //relativeTo

    /**
     * This method returns this pose expressed relative to the specified reference pose,
     * including transformation of its orientation.
     *
     * @param pose specifies the reference pose.
     * @return this pose expressed in the reference pose's coordinate frame.
     */
    public TrcPose3D relativeTo(TrcPose3D pose)
    {
        return relativeTo(pose, true);
    }   //relativeTo

    /**
     * Returns the rigid-body inverse of this pose.
     *
     * @return inverted pose.
     */
    public TrcPose3D inverse()
    {
        Rotation inverseRotation = getRotation().revert();
        Vector3D position = new Vector3D(x, y, z);
        Vector3D inversePosition = inverseRotation.applyTo(position).negate();
        TrcPose3D inversePose = new TrcPose3D(inversePosition.getX(), inversePosition.getY(), inversePosition.getZ());
        inversePose.setOrientation(inverseRotation);

        return inversePose;
    }   //inverse

    /**
     * Converts this pose's TRC pitch, roll and yaw to an Apache Commons Math
     * rotation.
     *
     * <p>TRC yaw is clockwise-positive while Apache Commons Math uses the
     * mathematical counter-clockwise-positive convention, so yaw is negated.
     */
    private Rotation getRotation()
    {
        return new Rotation(
            RotationOrder.XYZ,
            RotationConvention.VECTOR_OPERATOR,
            Math.toRadians(pitch),
            Math.toRadians(roll),
            Math.toRadians(-yaw));
    }   //getRotation

    /**
     * Creates an Apache rotation from the specified TRC pitch, roll and yaw
     * angles.
     *
     * @param pitch specifies the rotation angle around the X-axis in degrees.
     * @param roll specifies the rotation angle around the Y-axis in degrees.
     * @param yaw specifies the clockwise rotation angle around the Z-axis in degrees.
     * @return the corresponding Apache rotation.
     */
    private static Rotation getRotation(double pitch, double roll, double yaw)
    {
        return new Rotation(
            RotationOrder.XYZ,
            RotationConvention.VECTOR_OPERATOR,
            Math.toRadians(pitch),
            Math.toRadians(roll),
            Math.toRadians(-yaw));
    }   //getRotation

    /**
     * Extracts TRC pitch, roll and yaw from an Apache Commons Math rotation.
     */
    private void setOrientation(Rotation rotation)
    {
        double[] angles = rotation.getAngles(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR);

        pitch = Math.toDegrees(angles[0]);
        roll = Math.toDegrees(angles[1]);
        yaw = -Math.toDegrees(angles[2]);
    }   //setOrientation
}   //class TrcPose3D
