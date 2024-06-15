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

package trclib.sensor;

import java.util.Arrays;
import java.util.Locale;

import trclib.drivebase.TrcDriveBase;
import trclib.robotcore.TrcDbgTrace;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent drive base odometry device. A drive base odometry device generally
 * consists of two to five sensors: zero to two for the X-axis, one to two for the Y-axis and one rotational sensor.
 * All sensors must be able to report position as well as velocity data. If the sensor does not provide velocity
 * data natively, it must calculate velocity data by differentiating position data against time. For axis sensors,
 * they must be oriented parallel to the axis and installed tangential to the drive base centroid. They can have an
 * offset from the centroid of the drive base. For X-axis, the front sensor must have a positive offset value, the
 * back sensor must have a negative offset value. For Y-axis, the left sensor must have a positive offset value, the
 * right sensor must have a negative offset value. Some typical configurations are listed below.
 * <p>
 * Configuration 1:
 * This configuration has 2 sensors: one for Y-axis and one for rotation. The Y-axis sensor is most likely an encoder
 * on passive omni-wheels installed parallel to the Y-axis. The angle sensor is most likely a gyro. This configuration
 * does not support holonomic drive base since it doesn't provide X odometry. The Y sensor is typically installed
 * inline with either the left or the right wheels and tangential to the centroid of the drive base.
 * </p>
 * <p>
 * Configuration 2:
 * This configuration has 3 sensors: one for X-axis, one for Y-axis and one for rotation. The X and Y sensors are most
 * likely encoders on passive omni-wheels installed parallel to the X and Y axes respectively. The angle sensor is
 * most likely a gyro. This configuration can support holonomic drive base. The Y sensor is typically installed
 * inline with either the left or the right wheels and tangential to the drive base centroid. The X sensor is
 * typically installed on the front or the back of the drive base tangential to the centroid of the drive base.
 * </p>
 * <p>
 * Configuration 3:
 * This configuration also has 3 sensors: two for Y-axis and one for rotation. The Y sensors are most likely encoders
 * on passive omni-wheels installed parallel to the Y axis. The angle sensor is most likely a gyro. This configuration
 * does not support holonomic drive base since it doesn't provide X odometry. The two Y sensors are typically
 * installed on the left and right sides of the drive base equidistant to the centroid most likely inline with the
 * left and right wheels and tangential to the centroid of the drive base.
 * </p>
 * <p>
 * Configuration 4:
 * This configuration has 4 sensors: one for X-axis, two for Y-axis and one for rotation. The three axes sensors are
 * most likely encoders on passive omni-wheels installed parallel to the X and Y axes respectively. The angle sensor
 * is most likely a gyro. This configuration can support holonomic drive base. The two Y sensors are typically
 * installed on the left and right sides of the drive base equidistant to the centroid most likely inline with the
 * left and right wheels and tangential to the centroid of the drive base. The X sensor is typically installed on
 * the front or the back of the drive base and tangential to the centroid of the drive base.
 * </p>
 * <p>
 * Configuration 5:
 * This configuration has 5 sensors: two for X-axis, two for Y-axis and one for rotation. The four axes sensors are
 * most likely encoders on passive omni-wheels installed parallel to the X and Y axes respectively. The angle sensor
 * is most likely a gyro. This configuration can support holonmic drive base. The two Y sensors are typically
 * installed on the left and right sides of the drive base equidistant to the centroid most likely inline with the
 * left and right wheels and tangential to the centroid of the drive base. The two X sensors are typically installed
 * on the front or the back of the drive base equidistant to the centroid and tangential to the centroid of the drive
 * base.
 * </p>
 */
public class TrcOdometryWheels
{
    private static final String moduleName = TrcOdometryWheels.class.getSimpleName();

    /**
     * This class encapsulates an axis sensor with its axis offset.
     */
    public static class AxisSensor
    {
        // Odometry sensor, typically an encoder.
        final TrcOdometrySensor sensor;
        // Parallel offset of the axis wheel from wheel base centroid.
        final double axisOffset;
        // Clockwise angle offset in radians of the axis wheel from the tangent of the wheel base circle.
        final double angleOffset;
        private TrcOdometrySensor.Odometry odometry = null;

        /**
         * Constructor: Create an instance of the odometry wheel
         *
         * @param sensor specifies the odometry sensor.
         * @param parallelOffset specifies the distance offset of the line parallel to the odometry wheel from the
         *        wheel base centroid.
         * @param orthogonalOffset specifies the distance offset of the line orthogonal to the odometry wheel from
         *        wheel base centroid.
         */
        public AxisSensor(TrcOdometrySensor sensor, double parallelOffset, double orthogonalOffset)
        {
            this.sensor = sensor;
            // angle of the line parallel to the odometry wheel from the tangent of the turning circle. If the odometry
            // wheel is tangent to the turning circle, angleOffset is 0 degree.
            this.angleOffset = Math.atan(orthogonalOffset/parallelOffset);
            // axisOffset is the parallel offset adjusted by the angle offset.
            this.axisOffset = parallelOffset / Math.cos(angleOffset);
        }   //AxisSensor

        /**
         * Constructor: Create an instance of the object. The odometry wheel is tangent to the turning circle.
         *
         * @param sensor specifies the odometry sensor.
         * @param parallelOffset specifies the parallel axis offset from wheel base centroid.
         */
        public AxisSensor(TrcOdometrySensor sensor, double parallelOffset)
        {
            this(sensor, parallelOffset, 0.0);
        }   //AxisSensor

        /**
         * Constructor: Create an instance of the object. The odometry wheel is at the center of the turning circle.
         *
         * @param sensor specifies the odometry sensor.
         */
        public AxisSensor(TrcOdometrySensor sensor)
        {
            this(sensor, 0.0, 0.0);
        }   //AxisSensor

        /**
         * This method returns the sensor info in string form.
         *
         * @return string form of sensor info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "(%s: axisOffset=%f, angleOffset=%f, odometry=%s",
                sensor, axisOffset, angleOffset, odometry);
        }   //toString

    }   //class AxisSensor

    private final TrcDbgTrace tracer;
    private final AxisSensor[] xSensors;
    private final AxisSensor[] ySensors;
    private final TrcOdometrySensor angleSensor;
    private TrcOdometrySensor.Odometry angleOdometry;
    private double xScale = 1.0;
    private double yScale = 1.0;
    private double angleScale = 1.0;
    private double prevAvgXPos, prevAvgYPos;

    /**
     * Constructor: Create an instance of the object. This is typically used for configuration 5.
     *
     * @param xSensors specifies an array of Odometry sensors for the X-axis.
     * @param ySensors specifies an array of Odometry sensors for the Y-axis.
     * @param angleSensor specifies the Odometry sensor for rotation.
     */
    public TrcOdometryWheels(AxisSensor[] xSensors, AxisSensor[] ySensors, TrcOdometrySensor angleSensor)
    {
        if (ySensors == null || angleSensor == null || ySensors.length < 1)
        {
            throw new IllegalArgumentException("Must have at least one Y and an angle sensor.");
        }

        this.tracer = new TrcDbgTrace();
        this.xSensors = xSensors;
        this.ySensors = ySensors;
        this.angleSensor = angleSensor;
        //
        // Hardware reset all odometry sensors.
        //
        if (xSensors != null)
        {
            for (AxisSensor s: xSensors)
            {
                s.sensor.resetOdometry(true);
            }
        }

        for (AxisSensor s: ySensors)
        {
            s.sensor.resetOdometry(true);
        }

        angleSensor.resetOdometry(true);
    }   //TrcOdometryWheels

    /**
     * Constructor: Create an instance of the object. This is typically used for configuration 4.
     *
     * @param xSensor specifies an Odometry sensors for the X-axis.
     * @param ySensors specifies an array of Odometry sensors for the Y-axis.
     * @param angleSensor specifies the Odometry sensor for rotation.
     */
    public TrcOdometryWheels(AxisSensor xSensor, AxisSensor[] ySensors, TrcOdometrySensor angleSensor)
    {
        this(new AxisSensor[] {xSensor}, ySensors, angleSensor);
    }   //TrcOdometryWheels

    /**
     * Constructor: Create an instance of the object. This is typically used for configuration 3.
     *
     * @param ySensors specifies an array of Odometry sensors for the Y-axis.
     * @param angleSensor specifies the Odometry sensor for rotation.
     */
    public TrcOdometryWheels(AxisSensor[] ySensors, TrcOdometrySensor angleSensor)
    {
        this((AxisSensor[])null, ySensors, angleSensor);
    }   //TrcOdometryWheels

    /**
     * Constructor: Create an instance of the object. This is typically use for configuration 2.
     *
     * @param xSensor specifies an Odometry sensors for the X-axis.
     * @param ySensor specifies an Odometry sensors for the Y-axis.
     * @param angleSensor specifies the Odometry sensor for rotation.
     */
    public TrcOdometryWheels(AxisSensor xSensor, AxisSensor ySensor, TrcOdometrySensor angleSensor)
    {
        this(new AxisSensor[] {xSensor}, new AxisSensor[] {ySensor}, angleSensor);
    }   //TrcDriveBaseOdometry

    /**
     * Constructor: Create an instance of the object. This is typically use for configuration 1.
     *
     * @param ySensor specifies an Odometry sensors for the Y-axis.
     * @param angleSensor specifies the Odometry sensor for rotation.
     */
    public TrcOdometryWheels(AxisSensor ySensor, TrcOdometrySensor angleSensor)
    {
        this((AxisSensor[])null, new AxisSensor[] {ySensor}, angleSensor);
    }   //TrcDriveBaseOdometry

    /**
     * This method checks if the given sensor is used as part of the Odometry sensors.
     *
     * @param sensor specifies the sensor to be checked.
     * @return true if the sensor is part of the odometry sensors, false otherwise.
     */
    public boolean isSensorUsed(TrcOdometrySensor sensor)
    {
        boolean isUsed = false;

        if (xSensors != null)
        {
            for (AxisSensor s: xSensors)
            {
                if (sensor == s.sensor)
                {
                    isUsed = true;
                    break;
                }
            }
        }

        if (!isUsed && ySensors != null)
        {
            for (AxisSensor s: ySensors)
            {
                if (sensor == s.sensor)
                {
                    isUsed = true;
                    break;
                }
            }
        }

        if (!isUsed && angleSensor != null && sensor == angleSensor)
        {
            isUsed = true;
        }

        return isUsed;
    }   //isSensorUsed

    /**
     * This method sets the message trace level for the tracer.
     *
     * @param msgLevel specifies the message level.
     */
    public void setTraceLevel(TrcDbgTrace.MsgLevel msgLevel)
    {
        tracer.setTraceLevel(msgLevel);
    }   //setTraceLevel

    /**
     * This method sets the scaling factors for both X, Y and angle data. This is typically used to scale encoder
     * counts to physical units such as inches. If the scale of a direction is not provided, it must be set to 1.0.
     *
     * @param xScale specifies the scale factor for the X direction.
     * @param yScale specifies the scale factor for the Y direction.
     * @param angleScale specifies the scale factor the the angle.
     */
    public synchronized void setOdometryScales(double xScale, double yScale, double angleScale)
    {
        this.xScale = xScale;
        this.yScale = yScale;
        this.angleScale = angleScale;
    }   //setOdometryScales

    /**
     * This method sets the scaling factors for Y and angle data. This is typically used to scale encoder
     * counts to physical units such as inches. If the scale of a direction is not provided, it must be set to 1.0.
     *
     * @param xScale specifies the scale factor for the X direction.
     * @param yScale specifies the scale factor for the Y direction.
     */
    public void setOdometryScales(double xScale, double yScale)
    {
        this.setOdometryScales(xScale, yScale, 1.0);
    }   //setOdometryScales

    /**
     * This method resets the odometry device and data.
     *
     * @param resetPositionOdometry specifies true for resetting position odometry, false otherwise.
     *        Generally, this should be set to true unless the position odometry has been reset somewhere else.
     * @param resetHeadingOdometry specifies true to also reset the heading odometry, false otherwise.
     * @param resetHardware specifies true to do a hardware reset, false to do a soft reset.
     */
    public synchronized void resetOdometry(
        boolean resetPositionOdometry, boolean resetHeadingOdometry, boolean resetHardware)
    {
        prevAvgXPos = 0.0;
        if (xSensors != null)
        {
            for (AxisSensor s: xSensors)
            {
                if (resetPositionOdometry)
                {
                    s.sensor.resetOdometry(resetHardware);
                }
                s.odometry = s.sensor.getOdometry();
                prevAvgXPos += s.odometry.currPos;
            }
            prevAvgXPos /= xSensors.length;
        }

        prevAvgYPos = 0.0;
        if (ySensors != null)
        {
            for (AxisSensor s: ySensors)
            {
                if (resetPositionOdometry)
                {
                    s.sensor.resetOdometry(resetHardware);
                }
                s.odometry = s.sensor.getOdometry();
                prevAvgYPos += s.odometry.currPos;
            }
            prevAvgYPos /= ySensors.length;
        }

        if (resetHeadingOdometry && angleSensor != null)
        {
            angleSensor.resetOdometry(resetHardware);
            angleOdometry = angleSensor.getOdometry();
        }
    }   //resetOdometry

    /**
     * This method reads all the sensors and calculates the delta displacement from the last odometry update. Only
     * position data are deltas but not velocities because we only integrate position data into absolute field
     * position.
     *
     * @return delta odometry.
     */
    public synchronized TrcDriveBase.Odometry getOdometryDelta()
    {
        updateAxisOdometries(xSensors);
        updateAxisOdometries(ySensors);
        angleOdometry = angleSensor.getOdometry();

        double avgXPos = averageSensorValues(xSensors, xScale, true);
        double avgYPos = averageSensorValues(ySensors, yScale, true);
        double avgXVel = averageSensorValues(xSensors, xScale, false);
        double avgYVel = averageSensorValues(ySensors, yScale, false);
        //
        // Note: In odometryDelta, only position data is really a delta from previous position.
        // Velocity data IS NOT a delta.
        //
        TrcDriveBase.Odometry odometryDelta = new TrcDriveBase.Odometry();
        odometryDelta.position.x = (avgXPos - prevAvgXPos)*xScale;
        odometryDelta.position.y = (avgYPos - prevAvgYPos)*yScale;
        odometryDelta.position.angle = (angleOdometry.currPos - angleOdometry.prevPos)*angleScale;
        odometryDelta.velocity.x = avgXVel*xScale;
        odometryDelta.velocity.y = avgYVel*yScale;
        odometryDelta.velocity.angle = angleOdometry.velocity*angleScale;
        tracer.traceDebug(
            moduleName, "x=%s, y=%s, avgX=%f, avgY=%f, deltaX=%f, deltaY=%f, deltaAngle=%f",
            Arrays.toString(xSensors), Arrays.toString(ySensors), avgXPos, avgYPos, odometryDelta.position.x,
            odometryDelta.position.y, odometryDelta.position.angle);
        prevAvgXPos = avgXPos;
        prevAvgYPos = avgYPos;

        return odometryDelta;
    }   //getOdometryDelta

    /**
     * This method is called to update the odometry data for all sensors of the given axis.
     *
     * @param axisSensors specifies the axis sensor array to be updated.
     */
    private void updateAxisOdometries(AxisSensor[] axisSensors)
    {
        if (axisSensors != null)
        {
            for (AxisSensor s: axisSensors)
            {
                s.odometry = s.sensor.getOdometry();
            }
        }
    }   //updateAxisOdometries

    /**
     * This method calculates the average of either position or velocity odometry data of all sensors in the given
     * axis.
     *
     * @param axisSensors specifies the axis sensor array to be averaged.
     * @param scale specifies the odometry sensor scale.
     * @param position specifies true to average position data, false to average velocity data.
     * @return averaged value.
     */
    private double averageSensorValues(AxisSensor[] axisSensors, double scale, boolean position)
    {
        double sum = 0.0;

        for (int i = 0; i < axisSensors.length; i++)
        {
            AxisSensor s = axisSensors[i];
            double currData;

            if (position)
            {
                // Note: rotation doesn't generate any displacement and is therefore subtracted from the
                //       deltaDisplacement.
                // deltaDisplacement = deltaPos - rotationDistance
                //                   = (currPos - prevPos) - axisOffset * (currHeading - prevHeading)
                //                   = (currPos - axisOffset * currHeading) - (prevPos - axisOffset * prevHeading)
                //                   = currData - prevData
                currData = s.odometry.currPos - s.axisOffset/scale * Math.toRadians(angleOdometry.currPos);
            }
            else
            {
                // Note: rotation doesn't generate any displacement and therefore rotationVel is subtracted from
                //       the registered velocity.
                // displacementVel = currVel - rotationVel
                double deltaTime = angleOdometry.currTimestamp - angleOdometry.prevTimestamp;
                currData = s.odometry.velocity;
                if (deltaTime != 0)
                {
                    currData -=
                        s.axisOffset/scale * Math.toRadians(angleOdometry.currPos - angleOdometry.prevPos)/deltaTime;
                }
            }
            sum += currData;

            if (position)
            {
                tracer.traceDebug(
                    moduleName, "%s.Pos[%d] timestamp=%.6f, heading=%f, pos=%f, currData=%f, avgData=%f",
                    s.sensor.getName(), i, TrcTimer.getModeElapsedTime(s.odometry.currTimestamp),
                    angleOdometry.currPos, s.odometry.currPos, currData, sum/(i + 1));
            }
        }

        return sum/axisSensors.length;
    }   //averageSensorValues

}   //class TrcOdometryWheels
