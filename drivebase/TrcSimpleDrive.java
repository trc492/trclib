/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib.drivebase;

import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcGyro;
import trclib.motor.TrcMotor;
import trclib.sensor.TrcOdometrySensor;
import trclib.timer.TrcTimer;
import trclib.dataprocessor.TrcUtil;

/**
 * This class implements a platform independent simple drive base. The SimpleDriveBase class implements a drive train
 * that may consist of 2 to 6 motors. It supports tank drive, curve drive and arcade drive with motor stalled detection
 * and inverted drive mode. It also supports gyro assisted drive to keep robot driving straight.
 */
public class TrcSimpleDrive extends TrcDriveBase
{
    private final String moduleName = getClass().getSimpleName();

    /**
     * Constructor: Create an instance of a 6-wheel drive base.
     *
     * @param gyro specifies the gyro. If none, it can be set to null.
     * @param flMotor specifies the front left motor of the drive base.
     * @param frMotor specifies the front right motor of the drive base.
     * @param blMotor specifies the back left motor of the drive base.
     * @param brMotor specifies the back right motor of the drive base.
     * @param clMotor specifies the center left motor of a 6-wheel drive base.
     * @param crMotor specifies the center right motor of a 6-wheel drive base.
     */
    public TrcSimpleDrive(
        TrcGyro gyro, TrcMotor flMotor, TrcMotor frMotor, TrcMotor blMotor, TrcMotor brMotor,
        TrcMotor clMotor, TrcMotor crMotor)
    {
        super(gyro, flMotor, frMotor, blMotor, brMotor, clMotor, crMotor);

        if (flMotor == null || frMotor == null ||
            blMotor == null || brMotor == null ||
            clMotor == null || crMotor == null)
        {
            throw new IllegalArgumentException("All 6 motors must not be null.");
        }
    }   //TrcSimpleDrive

    /**
     * Constructor: Create an instance of a 6-wheel drive base.
     *
     * @param flMotor specifies the front left motor of the drive base.
     * @param frMotor specifies the front right motor of the drive base.
     * @param blMotor specifies the back left motor of the drive base.
     * @param brMotor specifies the back right motor of the drive base.
     * @param clMotor specifies the center left motor of a 6-wheel drive base.
     * @param crMotor specifies the center right motor of a 6-wheel drive base.
     */
    public TrcSimpleDrive(
        TrcMotor flMotor, TrcMotor frMotor, TrcMotor blMotor, TrcMotor brMotor, TrcMotor clMotor, TrcMotor crMotor)
    {
        this(null, flMotor, frMotor, blMotor, brMotor, clMotor, crMotor);
    }   //TrcSimpleDrive

    /**
     * Constructor: Create an instance of a 4-wheel drive base.
     *
     * @param gyro specifies the gyro. If none, it can be set to null.
     * @param flMotor specifies the front left motor of the drive base.
     * @param frMotor specifies the front right motor of the drive base.
     * @param blMotor specifies the back left motor of the drive base.
     * @param brMotor specifies the back right motor of the drive base.
     */
    public TrcSimpleDrive(
        TrcGyro gyro, TrcMotor flMotor, TrcMotor frMotor, TrcMotor blMotor, TrcMotor brMotor)
    {
        super(gyro, flMotor, frMotor, blMotor, brMotor);

        if (flMotor == null || frMotor == null || blMotor == null || brMotor == null)
        {
            throw new IllegalArgumentException("All 4 motors must not be null.");
        }
    }   //TrcSimpleDrive

    /**
     * Constructor: Create an instance of a 4-wheel drive base.
     *
     * @param flMotor specifies the front left motor of the drive base.
     * @param frMotor specifies the front right motor of the drive base.
     * @param blMotor specifies the back left motor of the drive base.
     * @param brMotor specifies the back right motor of the drive base.
     */
    public TrcSimpleDrive(
        TrcMotor flMotor, TrcMotor frMotor, TrcMotor blMotor, TrcMotor brMotor)
    {
        this(null, flMotor, frMotor, blMotor, brMotor);
    }   //TrcSimpleDrive

    /**
     * Constructor: Create an instance of a 2-wheel drive base.
     *
     * @param gyro specifies the gyro. If none, it can be set to null.
     * @param leftMotor specifies the left motor of the drive base.
     * @param rightMotor specifies the right motor of the drive base.
     */
    public TrcSimpleDrive(TrcGyro gyro, TrcMotor leftMotor, TrcMotor rightMotor)
    {
        super(gyro, leftMotor, rightMotor);

        if (leftMotor == null || rightMotor == null)
        {
            throw new IllegalArgumentException("All 2 motors must not be null.");
        }
    }   //TrcSimpleDrive

    /**
     * Constructor: Create an instance of a 2-wheel drive base.
     *
     * @param leftMotor specifies the left motor of the drive base.
     * @param rightMotor specifies the right motor of the drive base.
     */
    public TrcSimpleDrive(TrcMotor leftMotor, TrcMotor rightMotor)
    {
        this(null, leftMotor, rightMotor);
    }   //TrcSimpleDrive

    // CodeReview: Please explain what is this method for? Nobody is calling it. Why divide yScale by wheel base width?
    /**
     * This method sets the wheel base width of the robot drive base.
     *
     * @param width specifies the wheel base width.
     */
    public void setWheelBaseWidth(double width)
    {
        setOdometryScales(xScale, yScale, yScale / width);
    }   //setWheelBaseWidth

    /**
     * This method inverts direction of a given motor in the drive train.
     *
     * @param motorIndex specifies the motor index in the drive train.
     * @param inverted specifies true if inverting motor direction.
     */
    public void setInvertedMotor(MotorIndex motorIndex, boolean inverted)
    {
        setInvertedMotor(motorIndex.value, inverted);
    }   //setInvertedMotor

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *              ownership aware.
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    @Override
    public void tankDrive(
        String owner, double leftPower, double rightPower, boolean inverted, double driveTime, TrcEvent event)
    {
        tracer.traceDebug(
            moduleName,
            "owner=" + owner +
            ",leftPower=" + leftPower +
            ",rightPower=" + rightPower +
            ",inverted=" + inverted +
            ",driveTime=" + driveTime +
            ",event=" + event);
        if (validateOwnership(owner))
        {
            leftPower = TrcUtil.clipRange(leftPower);
            rightPower = TrcUtil.clipRange(rightPower);

            if (inverted)
            {
                double swap = leftPower;
                leftPower = -rightPower;
                rightPower = -swap;
            }

            if (isGyroAssistEnabled())
            {
                double assistPower = getGyroAssistPower((leftPower - rightPower)/2.0);
                leftPower += assistPower;
                rightPower -= assistPower;
            }

            if (isAntiTippingEnabled())
            {
                double antiTippingPower = getAntiTippingPower(false);
                leftPower += antiTippingPower;
                rightPower += antiTippingPower;
            }

            double maxMag = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxMag > 1.0)
            {
                leftPower /= maxMag;
                rightPower /= maxMag;
            }

            if (leftPower == 0.0 && rightPower == 0.0)
            {
                // reset stall start time to zero if drive base is stopped.
                stallStartTime = 0;
            }

            double wheelPower;
            TrcMotor motor;

            motor = motors[MotorIndex.FrontLeft.value];
            if (motor != null)
            {
                wheelPower = leftPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, motor.getVelocity());
                }
                motor.setPower(wheelPower);
            }

            motor = motors[MotorIndex.FrontRight.value];
            if (motor != null)
            {
                wheelPower = rightPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, motor.getVelocity());
                }
                motor.setPower(wheelPower);
            }

            motor = motors[MotorIndex.BackLeft.value];
            if (motor != null)
            {
                wheelPower = leftPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, motor.getVelocity());
                }
                motor.setPower(wheelPower);
            }

            motor = motors[MotorIndex.BackRight.value];
            if (motor != null)
            {
                wheelPower = rightPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, motor.getVelocity());
                }
                motor.setPower(wheelPower);
            }

            motor = motors[MotorIndex.CenterLeft.value];
            if (motor != null)
            {
                wheelPower = leftPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, motor.getVelocity());
                }
                motor.setPower(wheelPower);
            }

            motor = motors[MotorIndex.CenterRight.value];
            if (motor != null)
            {
                wheelPower = rightPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, motor.getVelocity());
                }
                motor.setPower(wheelPower);
            }
            setDriveTime(owner, driveTime, event);
        }
    }   //tankDrive

    /**
     * This method is called periodically to calculate the delta between the previous and current motor odometries.
     *
     * @param prevOdometries specifies the previous motor odometries.
     * @param currOdometries specifies the current motor odometries.
     * @return an Odometry object describing the odometry changes since the last update.
     */
    @Override
    protected Odometry getOdometryDelta(
        TrcOdometrySensor.Odometry[] prevOdometries, TrcOdometrySensor.Odometry[] currOdometries)
    {
        Odometry delta = new Odometry();

        //
        // Calculate heading and turn rate using positional info in case we don't have a gyro.
        // Get the average of all left and right motors separately, since this drivebase may have between 2-6 motors
        //
        double lPos = 0.0, rPos = 0.0;
        double lVel = 0.0, rVel = 0.0;

        for (int i = 0; i < currOdometries.length; i++)
        {
            double posDelta = currOdometries[i].currPos - prevOdometries[i].currPos;
            double vel = currOdometries[i].velocity;

            if (i % 2 == 0)
            {
                lPos += posDelta;
                lVel += vel;
            }
            else
            {
                rPos += posDelta;
                rVel += vel;
            }
        }

        double motorsPerSide = getNumMotors() / 2.0;
        lPos /= motorsPerSide;
        rPos /= motorsPerSide;
        lVel /= motorsPerSide;
        rVel /= motorsPerSide;

        delta.position.x = 0.0;
        delta.position.y = (lPos + rPos)/2 * yScale;

        delta.velocity.x = 0.0;
        delta.velocity.y = (lVel + rVel)/2 * yScale;

        delta.position.angle = (lPos - rPos)/2 * angleScale;
        delta.velocity.angle = (lVel - rVel)/2 * angleScale;

        if (Math.abs(delta.velocity.y) > stallVelThreshold)
        {
            // reset stall start time to current time if drive base has movement.
            stallStartTime = TrcTimer.getCurrentTime();
        }

        return delta;
    }   //getOdometryDelta

}   //class TrcSimpleDrive
