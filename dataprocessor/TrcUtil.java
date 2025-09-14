/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib.dataprocessor;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.Arrays;

import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;

/**
 * This class contains platform independent utility methods. All methods in this class are static. It is not
 * necessary to instantiate this class to call its methods.
 */
public class TrcUtil
{
    public static final double MM_PER_INCH = 25.4;
    public static final double METERS_PER_INCH = MM_PER_INCH / 1000.0;
    public static final double INCHES_PER_MM = 1.0 / MM_PER_INCH;
    public static final double INCHES_PER_CM = 10.0 / MM_PER_INCH;
    public static final double INCHES_PER_METER = 1000.0 / MM_PER_INCH;
    public static final double EARTH_GRAVITATIONAL_CONSTANT = 9.807;    //in m/s2
    public static final double BATTERY_NOMINAL_VOLTAGE = 12.0;          //in volts

//    /**
//     * This method loads data points from a CSV file either on the external file system or attached resources.
//     *
//     * @param path specifies the file system path or resource name.
//     * @param numElements specifies the number of elements in a data point.
//     * @param loadFromResources specifies true if the data is from attached resources, false if from file system.
//     * @return an array of data points.
//     */
//    public static <T> T[] loadDataPointsFromCsv(String path, int numElements, boolean loadFromResources)
//    {
//        T[] dataPoints = null;
//
//        if (!path.endsWith(".csv"))
//        {
//            throw new IllegalArgumentException(path + " is not a csv file!");
//        }
//
//        try
//        {
//            BufferedReader in = new BufferedReader(
//                loadFromResources?
//                    new InputStreamReader(TrcUtil.class.getClassLoader().getResourceAsStream(path)):
//                    new FileReader(path));
//            List<T> pointList = new ArrayList<>();
//            String line;
//
//            in.readLine();  // Get rid of the first header line
//            while ((line = in.readLine()) != null)
//            {
//                String[] tokens = line.split(",");
//
//                if (tokens.length != numElements)
//                {
//                    throw new IllegalArgumentException("There must be " + numElements + " columns in the csv file!");
//                }
//
//                double[] elements = new double[tokens.length];
//
//                for (int i = 0; i < elements.length; i++)
//                {
//                    elements[i] = Double.parseDouble(tokens[i]);
//                }
//
//                Class<T> tClass;
//                T point = tClass.newInstance();
//
//                pointList.add(point);
//            }
//            in.close();
//
//            dataPoints = pointList.toArray(new T[0]);
//        }
//        catch (IOException e)
//        {
//            throw new RuntimeException(e);
//        }
//
//        return dataPoints;
//    }   //loadPointsFromCsv

    /**
     * This method calculates the modulo of two numbers. Unlike the <code>%</code> operator, this returns a number
     * in the range [0, b). For some reason, in Java, the <code>%</code> operator actually does remainder, which
     * means the result is in the range (-b, b).
     *
     * @param a specifies the dividend.
     * @param b specifies the divisor.
     * @return the modulo in the range [0, b)
     */
    public static double modulo(double a, double b)
    {
        return ((a % b) + b) % b;
    }   //modulo

    /**
     * This method sums an array of numbers.
     *
     * @param nums specifies the array of numbers to be summed.
     * @return sum of the numbers.
     */
    public static double sum(double... nums)
    {
        double total = 0.0;

        for (double num : nums)
        {
            total += num;
        }

        return total;
    }   //sum

    /**
     * This method calculates and returns the median of the numbers in the given array.
     *
     * @param num specifies the number array.
     * @return median of numbers in the array.
     */
    public static double median(double... num)
    {
        double m = 0.0;

        if (num.length > 0)
        {
            double[] nums = num.clone();

            Arrays.sort(nums);
            if (nums.length % 2 == 0)
            {
                m = TrcUtil.average(nums[(nums.length / 2) - 1], nums[nums.length / 2]);
            }
            else
            {
                m = nums[nums.length / 2];
            }
        }

        return m;
    }   //median

    /**
     * This method calculates and returns the average of the numbers in the given array.
     *
     * @param nums specifies the number array.
     * @return average of all numbers in the array. If the array is empty, return 0.
     */
    public static double average(double... nums)
    {
        return nums.length == 0 ? 0.0 : sum(nums) / nums.length;
    }   //average

    /**
     * This method calculates the magnitudes of the given array of numbers.
     *
     * @param nums specifies the number array.
     * @return magnitude of all numbers in the array.
     */
    public static double magnitude(double... nums)
    {
        double total = 0.0;

        for (double num : nums)
        {
            total += num * num;
        }

        return Math.sqrt(total);
        // return Math.sqrt(Arrays.stream(nums).map(e -> e*e).sum());
    }   //magnitude

    /**
     * This method returns the maximum magnitude of numbers in the specified array.
     *
     * @param nums specifies the number array.
     * @return maximum magnitude of the numbers in the array.
     */
    public static double maxMagnitude(double... nums)
    {
        double maxMag = Math.abs(nums[0]);

        for (double num : nums)
        {
            double magnitude = Math.abs(num);
            if (magnitude > maxMag)
            {
                maxMag = magnitude;
            }
        }

        return maxMag;
    }   //maxMagnitude

    /**
     * This method returns a bit mask of the least significant set bit.
     *
     * @param data specifies the data to find the least significant set bit.
     * @return bit mask that has only the least significant set bit.
     */
    public static int leastSignificantSetBit(int data)
    {
        return Integer.lowestOneBit(data);  // basically data & -data
    }   //leastSignificantSetBit

    /**
     * This method returns the bit position of the least significant set bit of the given data.
     *
     * @param data specifies the data to determine its least significant set bit position.
     * @return 0-based least significant set bit position. -1 if no set bit.
     */
    public static int leastSignificantSetBitPosition(int data)
    {
        int pos = -1;

        if (data != 0)
        {
            pos = Integer.numberOfTrailingZeros(data);
        }

        return pos;
    }   //leastSignificantSetBitPosition

    /**
     * This method returns a bit mask of the least significant set bit.
     *
     * @param data specifies the data to find the least significant set bit.
     * @return bit mask that has only the least significant set bit.
     */
    public static int mostSignificantSetBit(int data)
    {
        // //
        // // Duplicate the most significant set bit to all bits to the right.
        // // Add one to result so all set bits are clear except one bit to the left of the most significant set bit.
        // // Shift one bit to the right to adjust to final bit position.
        // //
        // if (data != 0)
        // {
        //     data |= data >>> 1;
        //     data |= data >>> 2;
        //     data |= data >>> 4;
        //     data |= data >>> 8;
        //     data |= data >>> 16;
        //     data += 1;
        //     // Take care of the sign bit corner case.
        //     data = data == 0? 0x80000000: data >>> 1;
        // }
        //
        // return data;

        return Integer.highestOneBit(data);
    }   //mostSignificantSetBit

    /**
     * This method returns the bit position of the most significant set bit of the given data.
     *
     * @param data specifies the data to determine its most significant set bit position.
     * @return 0-based most significant set bit position. -1 if no set bit.
     */
    public static int mostSignificantSetBitPosition(int data)
    {
        int pos = -1;

        if (data != 0)
        {
            pos = 31 - Integer.numberOfLeadingZeros(data);
        }

        return pos;
    }   //mostSignificantSetBitPosition

    /**
     * This method sets a bitmask with the given bit positions.
     *
     * @param bitPositions specifies the bit positions to be set to 1. Bit positions are 0-based.
     * @return bit mask.
     */
    public static int setBitMask(int... bitPositions)
    {
        int bitMask = 0;

        for (int pos : bitPositions)
        {
            bitMask |= 1 << pos;
        }

        return bitMask;
    }   //setBitMask

    /**
     * This method normalizes the given array of numbers such that no number exceeds +/- 1.0. If no number exceeds
     * the magnitude of 1.0, nothing will change, otherwise the original nums array will be modified in place.
     *
     * @param nums specifies the number array.
     */
    public static void normalizeInPlace(double[] nums)
    {
        double maxMag = maxMagnitude(nums);

        if (maxMag > 1.0)
        {
            for (int i = 0; i < nums.length; i++)
            {
                nums[i] /= maxMag;
            }
        }
    }   //normalizeInPlace

    /**
     * This method normalizes the given array of numbers such that no number exceeds +/- 1.0.
     *
     * @param nums specifies the number array.
     * @return normalized number array.
     */
    public static double[] normalize(double... nums)
    {
        double[] result = nums.clone();
        normalizeInPlace(result);

        return result;
    }   //normalize

    /**
     * This method rounds a double to the nearest integer.
     *
     * @param num specifies the number to round.
     * @return number rounded to the nearest integer.
     */
    public static int round(double num)
    {
        return (int) Math.floor(num + 0.5);
    }   //round

    /**
     * This method rounds a double to the specified precision.
     *
     * @param num specifies the number to round.
     * @param precision specifies the precision to round to.
     * @return number rounded to the specified precision.
     */
    public static double round(double num, double precision)
    {
        return Math.round(num / precision) * precision;
    }   //round

    /**
     * This method checks if the given value is within the specified range.
     *
     * @param value     The value to be checked.
     * @param low       The low limit of the range.
     * @param high      The high limit of the range.
     * @param inclusive specifies true if the range is inclusive [low, high], otherwise the range is exclusive (low, high).
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(int value, int low, int high, boolean inclusive)
    {
        return inclusive ? value >= low && value <= high : value > low && value < high;
    }   //inRange

    /**
     * This method checks if the given value is within the specified range inclusive.
     *
     * @param value The value to be checked.
     * @param low   The low limit of the range.
     * @param high  The high limit of the range.
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(int value, int low, int high)
    {
        return inRange(value, low, high, true);
    }   //inRange

    /**
     * This method checks if the given value is within the specified range.
     *
     * @param value     The value to be checked.
     * @param low       The low limit of the range.
     * @param high      The high limit of the range.
     * @param inclusive specifies true if the range is inclusive [low, high], otherwise the range is exclusive (low,high).
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(double value, double low, double high, boolean inclusive)
    {
        return inclusive ? value >= low && value <= high : value > low && value < high;
    }   //inRange

    /**
     * This method checks if the given value is within the specified range inclusive.
     *
     * @param value The value to be checked.
     * @param low   The low limit of the range.
     * @param high  The high limit of the range.
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(double value, double low, double high)
    {
        return inRange(value, low, high, true);
    }   //inRange

    /**
     * This method clips the given value to the range limited by the given low and high limits.
     *
     * @param value     specifies the value to be clipped
     * @param lowLimit  specifies the low limit of the range.
     * @param highLimit specifies the high limit of the range.
     * @return the result of the clipped value.
     */
    public static int clipRange(int value, int lowLimit, int highLimit)
    {
        return Math.min(Math.max(value, lowLimit), highLimit);
    }   //clipRange

    /**
     * This method clips the given value to the range limited by the given low and high limits.
     *
     * @param value     specifies the value to be clipped
     * @param lowLimit  specifies the low limit of the range.
     * @param highLimit specifies the high limit of the range.
     * @return the result of the clipped value.
     */
    public static double clipRange(double value, double lowLimit, double highLimit)
    {
        return Math.min(Math.max(value, lowLimit), highLimit);
    }   //clipRange

    /**
     * This method clips the given value to the range limited by -limit as the low limit and limit as the high limit.
     *
     * @param value     specifies the value to be clipped
     * @param limit     specifies the limit of the range between -limit and limit.
     * @return the result of the clipped value.
     */
    public static double clipRange(double value, double limit)
    {
        return clipRange(value, -limit, limit);
    }   //clipRange

    /**
     * This method clips the given value to the range between -1.0 and 1.0.
     *
     * @param value specifies the value to be clipped
     * @return the result of the clipped value.
     */
    public static double clipRange(double value)
    {
        return clipRange(value, -1.0, 1.0);
    }   //clipRange

    /**
     * This method scales the given value from the source range to the target range.
     *
     * @param value        specifies the value to be scaled.
     * @param lowSrcRange  specifies the low limit of the source range.
     * @param highSrcRange specifies the high limit of the source range.
     * @param lowDstRange  specifies the low limit of the target range.
     * @param highDstRange specifies the high limit of the target range
     * @return the result of the scaled value.
     */
    public static int scaleRange(int value, int lowSrcRange, int highSrcRange, int lowDstRange, int highDstRange)
    {
        return lowDstRange + (value - lowSrcRange) * (highDstRange - lowDstRange) / (highSrcRange - lowSrcRange);
    }   //scaleRange

    /**
     * This method scales the given value from the source range to the target range.
     *
     * @param value        specifies the value to be scaled.
     * @param lowSrcRange  specifies the low limit of the source range.
     * @param highSrcRange specifies the high limit of the source range.
     * @param lowDstRange  specifies the low limit of the target range.
     * @param highDstRange specifies the high limit of the target range
     * @return the result of the scaled value.
     */
    public static double scaleRange(double value, double lowSrcRange, double highSrcRange, double lowDstRange,
        double highDstRange)
    {
        return lowDstRange + (value - lowSrcRange) * (highDstRange - lowDstRange) / (highSrcRange - lowSrcRange);
    }   //scaleRange

    /**
     * This method checks if the given value is within the deadband range. If so, it returns 0.0 else it returns
     * the unchanged value.
     *
     * @param value    specifies the value to be checked.
     * @param deadband specifies the deadband zone.
     * @return the value 0.0 if within deadband, unaltered otherwise.
     */
    public static double applyDeadband(double value, double deadband)
    {
        return Math.abs(value) >= deadband ? value : 0.0;
    }   //applyDeadband

    /**
     * This method returns the indexed byte of an integer.
     *
     * @param data  specifies the integer value.
     * @param index specifies the byte index.
     * @return indexed byte of the integer.
     */
    public static byte intToByte(int data, int index)
    {
        return (byte) (data >> (8 * index));
    }   //intToByte

    /**
     * This method combines two bytes into an integer.
     *
     * @param data1 specifies the lowest byte.
     * @param data2 specifies the next lowest byte.
     * @param data3 specifies the next byte.
     * @param data4 specifies the highest byte.
     * @return the converted integer.
     */
    public static int bytesToInt(byte data1, byte data2, byte data3, byte data4)
    {
        return (data4 << 24) & 0xff000000 | (data3 << 16) & 0x00ff0000 | (data2 << 8) & 0x0000ff00 | data1 & 0x000000ff;
    }   //bytesToInt

    /**
     * This method combines two bytes into an integer.
     *
     * @param low  specifies the low byte.
     * @param high specifies the high byte.
     * @return the converted integer.
     */
    public static int bytesToInt(byte low, byte high)
    {
        return bytesToInt(low, high, (byte) 0, (byte) 0);
    }   //bytesToInt

    /**
     * This method converts a byte into an integer.
     *
     * @param data specifies the byte data.
     * @return the converted integer.
     */
    public static int bytesToInt(byte data)
    {
        return data;
    }   //bytesToInt

    /**
     * This method combines two bytes into a short.
     *
     * @param low  specifies the low byte.
     * @param high specifies the high byte.
     * @return the converted short.
     */
    public static short bytesToShort(byte low, byte high)
    {
        return (short) bytesToInt(low, high);
    }   //bytesToShort

    /**
     * Convert a point from a polar coordinate system to a cartesian coordinate system.
     *
     * @param r     Magnitude of vector
     * @param theta Direction of vector, in degrees clockwise from 0 (+y)
     * @return Vector in a cartesian coordinate system representing the same point.
     */
    public static RealVector polarToCartesian(double r, double theta)
    {
        double thetaRad = Math.toRadians(theta);
        return MatrixUtils.createRealVector(new double[] { r * Math.sin(thetaRad), r * Math.cos(thetaRad) });
    }   //polarToCartesian

    /**
     * Create an {@link ArrayRealVector} with the supplied numbers. This is just quality of life to reduce typing.
     *
     * @param vector The data to put in the vector.
     * @return A new vector instance with the data.
     */
    public static RealVector createVector(double... vector)
    {
        return new ArrayRealVector(vector);
    } //createVector

    /**
     * Rotate a point counter-clockwise about the origin.
     *
     * @param vector The vector to rotate.
     * @param angle  The angle in degrees to rotate by.
     * @return The vector after the rotation transformation.
     */
    public static RealVector rotateCCW(RealVector vector, double angle)
    {
        return createCCWRotationMatrix(angle).operate(vector);
    }   //rotateCCW

    /**
     * Rotate a point clockwise about the origin.
     *
     * @param vector The vector to rotate.
     * @param angle  The angle in degrees to rotate by.
     * @return The vector after the rotation transformation.
     */
    public static RealVector rotateCW(RealVector vector, double angle)
    {
        return createCWRotationMatrix(angle).operate(vector);
    }   //rotateCW

    /**
     * Create a rotation matrix that will rotate a point counter-clockwise
     * about the origin by a specific number of degrees.
     *
     * @param angle The angle in degrees to rotate by.
     * @return A rotation matrix describing a counter-clockwise rotation by <code>angle</code> degrees.
     */
    public static RealMatrix createCCWRotationMatrix(double angle)
    {
        double angleRad = Math.toRadians(angle);
        return MatrixUtils.createRealMatrix(
            new double[][] { { Math.cos(angleRad), -Math.sin(angleRad) }, { Math.sin(angleRad), Math.cos(angleRad) } });
    }   //createCCWRotationMatrix

    /**
     * Create a rotation matrix that will rotate a point clockwise
     * about the origin by a specific number of degrees.
     *
     * @param angle The angle in degrees to rotate by.
     * @return A rotation matrix describing a clockwise rotation by <code>angle</code> degrees.
     */
    public static RealMatrix createCWRotationMatrix(double angle)
    {
        return createCCWRotationMatrix(angle).transpose();
    }   //createCWRotationMatrix

    /**
     * Projects a target point in camera space onto the floor (Z=0 plane),
     * returning coordinates in the robot's frame (robot at 0,0, heading=0).
     *
     * @param targetCamera Target position in camera space.
     * @param cameraPose Camera pose relative to robot origin (TrcPose3D), includes offset (x,y,z) and orientation
     *        (roll,pitch,yaw).
     * @return TrcPose2D of projected floor point in robot coordinates, or null if no intersection.
     */
    public static TrcPose2D projectToFloorRobot(Vector3D targetCamera, TrcPose3D cameraPose)
    {
        // Build camera rotation relative to robot (Yaw = heading around Z, Pitch = around Y, Roll = around X)
        Rotation camRot = new Rotation(
            RotationOrder.ZYX, RotationConvention.FRAME_TRANSFORM, cameraPose.yaw, cameraPose.pitch, cameraPose.roll);
        // Transform target from camera -> robot space
        Vector3D targetRobot =
            camRot.applyTo(targetCamera).add(new Vector3D(cameraPose.x, cameraPose.y, cameraPose.z));
        // Camera origin in robot space
        Vector3D cameraRobot = new Vector3D(cameraPose.x, cameraPose.y, cameraPose.z);
        // Project line from camera to target onto Z=0 plane
        double dz = targetRobot.getZ() - cameraRobot.getZ();

        if (Math.abs(dz) < 1e-6)
        {
            return null; // Parallel to floor
        }

        double scale = -cameraRobot.getZ() / dz;
        double floorX = cameraRobot.getX() + scale * (targetRobot.getX() - cameraRobot.getX());
        double floorY = cameraRobot.getY() + scale * (targetRobot.getY() - cameraRobot.getY());
        // Heading from robot to floor point
        double headingToTarget = Math.atan2(floorY, floorX);

        return new TrcPose2D(floorX, floorY, headingToTarget);
    }   //projectToFloorRobot

}   //class TrcUtil
