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
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import trclib.dataprocessor.TrcUtil;

/**
 * This class implements a 3D pose object that represents the positional and orientation state of an object.
 */

public class TrcPose3D
{
    public double x;
    public double y;
    public double z;
    public double pitch; // Rotation around X-axis
    public double roll;  // Rotation around Y-axis
    public double yaw;   // Rotation around Z-axis

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x component of the pose.
     * @param y specifies the y component of the pose.
     * @param z specifies the z component of the pose.
     * @param pitch specifies the pitch angle (rotation on X axis).
     * @param roll specifies the roll angle (rotation on Y axis).
     * @param yaw specifies the yaw angle (rotation on Z axis).
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
        return "(x=" + x + ",y=" + y + ",z=" + z + ",pitch=" + pitch + ",roll=" + roll + ",yaw=" + yaw + ")";
    }   //toString

    /**
     * This method creates TRC 3D poses from a CSV file.
     *
     * @param path specifies resource stream name or CSV file path.
     * @param loadFromResources specifies true if path is a resource stream name, false if it is a CSV file path.
     * @return array of TrcPose3D poses.
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
                    new InputStreamReader(
                        Objects.requireNonNull(TrcPose3D.class.getClassLoader()).getResourceAsStream(path)):
                    new FileReader(path));
            List<TrcPose3D> poseList = new ArrayList<>();
            String line;

            in.readLine();  // Get rid of header
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
     * This method compares the given pose with this one.
     *
     * @param o specifies the pose to compare to.
     * @return true if they are equal, false otherwise.
     */
    @Override
    public boolean equals(Object o)
    {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        TrcPose3D pose = (TrcPose3D) o;

        return Double.compare(pose.x, x) == 0 &&
               Double.compare(pose.y, y) == 0 &&
               Double.compare(pose.z, z) == 0 &&
               Double.compare(pose.pitch, pitch) == 0 &&
               Double.compare(pose.roll, roll) == 0 &&
               Double.compare(pose.yaw, yaw) == 0;
    }   //equals

    /**
     * This method computes the hashcode of this pose.
     *
     * @return computed hashcode.
     */
    @Override
    public int hashCode()
    {
        return Objects.hash(x, y, z, pitch, roll, yaw);
    }   //hashCode

    /**
     * This method returns a cloned copy of this pose.
     *
     * @return cloned pose.
     */
    @Override
    public TrcPose3D clone()
    {
        return new TrcPose3D(this.x, this.y, this.z, this.pitch, this.roll, this.yaw);
    }   //clone

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
     * This method returns the vector form of this pose.
     *
     * @return vector form of this pose.
     */
    public RealVector toPosVector()
    {
        return TrcUtil.createVector(x, y, z);
    }   //toPosVector

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
        this.pitch = pose.pitch;
        this.roll = pose.roll;
        this.yaw = pose.yaw;
    }   //setAs

    /**
     * This method performs addition translation to this pose.
     *
     * @param pose specifies the translation pose to be added to this pose.
     * @return resulting pose.
     */
    public TrcPose3D add(TrcPose3D pose)
    {
        return new TrcPose3D(this.x + pose.x, this.y + pose.y, this.z + pose.z, this.pitch, this.roll, this.yaw);
    }   //add

    /**
     * This method performs subtraction translation to this pose.
     *
     * @param pose specifies the translation pose to be subtracted to this pose.
     * @return resulting pose.
     */
    public TrcPose3D subtract(TrcPose3D pose)
    {
        return new TrcPose3D(this.x - pose.x, this.y - pose.y, this.z - pose.z, this.pitch, this.roll, this.yaw);
    }   //subtract

    /**
     * This method negate this pose.
     *
     * @return resulting pose.
     */
    public TrcPose3D negate()
    {
        return new TrcPose3D(-this.x, -this.y, -this.z, this.pitch, this.roll, this.yaw);
    }   //negate

    /**
     * This method scales this pose with the given scale factor.
     *
     * @param factor specifies the scaling factor.
     * @return resulting pose.
     */
    public TrcPose3D scale(double factor)
    {
        return new TrcPose3D(this.x * factor, this.y * factor, this.z * factor, this.pitch, this.roll, this.yaw);
    }   //scale

    /**
     * This method applies a 3D rotation matrix rotation to the positional coordinates.
     * By utilizing RotationOrder.XYZ, Apache rotates around X (alpha1), then Y (alpha2), then Z (alpha3).
     * This matches the pitch, roll, yaw variable order perfectly.
     *
     * @param rotationPose specifies the rotation to be performed.
     * @return resulting pose.
     */
    public TrcPose3D rotate(TrcPose3D rotationPose)
    {
        Rotation rot = new Rotation(
            RotationOrder.ZXY, // Configured to handle primary Z-Yaw transitions first
            RotationConvention.VECTOR_OPERATOR,
            Math.toRadians(-rotationPose.yaw),   // alpha1 -> Z axis (negated for CW positive)
            Math.toRadians(rotationPose.pitch),  // alpha2 -> X axis
            Math.toRadians(rotationPose.roll)    // alpha3 -> Y axis
        );

        Vector3D posVec = new Vector3D(this.x, this.y, this.z);
        Vector3D rotatedVec = rot.applyTo(posVec);

        return new TrcPose3D(
            rotatedVec.getX(), rotatedVec.getY(), rotatedVec.getZ(), this.pitch, this.roll, this.yaw);
    }   //rotate

    /**
     * This method translates this pose with the given offsets mapped relative to its current 3D orientation.
     *
     * @param xOffset specifies the x offset relative to the pose's orientation.
     * @param yOffset specifies the y offset relative to the pose's orientation.
     * @param zOffset specifies the z offset relative to the pose's orientation.
     * @return translated pose.
     */
    public TrcPose3D translatePose(double xOffset, double yOffset, double zOffset)
    {
        // Re-use your robust 3D rotation logic by treating the offset as a temporary relative pose
        TrcPose3D offsetPose = new TrcPose3D(xOffset, yOffset, zOffset, 0, 0, 0);
        TrcPose3D rotatedOffset = offsetPose.rotate(this);

        // Combine translated offset with current position
        return this.add(rotatedOffset);
    }   //translatePose

    /**
     * This method adds a relative pose to this pose and returns the resulting global pose.
     * The relative pose's translation is rotated by this pose's orientation, and its
     * orientation is compounded mathematically via 3D matrix multiplication.
     *
     * @param relativePose specifies the pose relative to this pose.
     * @return resulting global pose with precise 3D orientation.
     */
    public TrcPose3D addRelativePose(TrcPose3D relativePose)
    {
        // Transform the spatial translation vector
        TrcPose3D rotatedRelativeOffset = relativePose.rotate(this);
        TrcPose3D finalPose = this.add(rotatedRelativeOffset);
        // Compound the 3D orientations properly using matrix multiplication
        Rotation currentRot = this.getRotation();
        Rotation relativeRot = relativePose.getRotation();
        // Compound transformations: Apply current orientation first, then relative orientation
        Rotation compoundedRot = relativeRot.applyTo(currentRot);

        // Extract the clean combined angles back into our convention
        finalPose.setOrientation(compoundedRot);

        return finalPose;
    }   //addRelativePose


        // Compound transformations: Reverse compounding order for VECTOR_OPERATOR convention

    /**
     * This method returns a transformed pose relative to the given reference pose.
     * Properly calculates the orientation difference via matrix composition.
     *
     * @param pose specifies the reference frame pose.
     * @param transformAngle specifies true to also transform orientation, false to keep this pose's orientation.
     * @return pose relative to the given reference pose.
     */
    public TrcPose3D relativeTo(TrcPose3D pose, boolean transformAngle)
    {
        // Calculate linear delta in world coordinates and un-rotate it into the local frame
        double deltaX = this.x - pose.x;
        double deltaY = this.y - pose.y;
        double deltaZ = this.z - pose.z;
        // Invert the reference frame's rotation matrix
        Rotation refRotInverse = pose.getRotation().revert();
        Vector3D deltaVec = new Vector3D(deltaX, deltaY, deltaZ);
        Vector3D localVec = refRotInverse.applyTo(deltaVec);
        TrcPose3D relativePose = new TrcPose3D(localVec.getX(), localVec.getY(), localVec.getZ());

        // Handle true 3D orientation subtraction if requested
        if (transformAngle)
        {
            // Find the relative rotation: R_rel = R_ref^(-1) * R_this
            Rotation relativeRot = refRotInverse.applyTo(this.getRotation());
            relativePose.setOrientation(relativeRot);
        }
        else
        {
            relativePose.pitch = this.pitch;
            relativePose.roll  = this.roll;
            relativePose.yaw   = this.yaw;
        }

        return relativePose;
    }   //relativeTo

    /**
     * This method converts this pose's pitch, roll, and yaw into an Apache Rotation object.
     * Takes care of sign conventions (negating your CW-positive yaw to match standard CCW math).
     */
    private Rotation getRotation()
    {
        return new Rotation(
            RotationOrder.XYZ,
            RotationConvention.VECTOR_OPERATOR,
            Math.toRadians(this.pitch),
            Math.toRadians(this.roll),
            Math.toRadians(-this.yaw) // Negate to handle CW positive convention
        );
    }   //getRotation

    /**
     * This method extracts and sets pitch, roll, and yaw angles from an Apache Rotation object.
     * Restores CW-positive yaw convention.
     */
    private void setOrientation(Rotation rotation)
    {
        double[] angles = rotation.getAngles(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR);
        this.pitch = Math.toDegrees(angles[0]);
        this.roll  = Math.toDegrees(angles[1]);
        this.yaw   = -Math.toDegrees(angles[2]); // Re-negate to restore CW positive convention
    }   //setOrientation

}   //class TrcPose3D
