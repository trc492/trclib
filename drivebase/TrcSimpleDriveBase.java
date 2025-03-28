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
import trclib.pathdrive.TrcPose2D;
import trclib.sensor.TrcOdometrySensor;
import trclib.timer.TrcTimer;
import trclib.dataprocessor.TrcUtil;

/**
 * This class implements a platform independent simple drive base. The SimpleDriveBase class implements a drive train
 * that may consist of 2 to 6 motors. It supports tank drive, curve drive and arcade drive with motor stalled detection
 * and inverted drive mode. It also supports gyro assisted drive to keep robot driving straight.
 */
public class TrcSimpleDriveBase extends TrcDriveBase
{
    private final String moduleName = getClass().getSimpleName();

    public enum MotorType
    {
        LEFT_FRONT(0),
        RIGHT_FRONT(1),
        LEFT_BACK(2),
        RIGHT_BACK(3),
        LEFT_CENTER(4),
        RIGHT_CENTER(5);

        public final int value;

        MotorType(int value)
        {
            this.value = value;
        }
    }   //enum MotorType

    protected final TrcMotor lfMotor;
    protected final TrcMotor rfMotor;
    protected final TrcMotor lbMotor;
    protected final TrcMotor rbMotor;
    protected final TrcMotor lcMotor;
    protected final TrcMotor rcMotor;

    /**
     * Constructor: Create an instance of a 6-wheel drive base.
     *
     * @param lfMotor specifies the left front motor of the drive base.
     * @param lcMotor specifies the left center motor of a 6-wheel drive base.
     * @param lbMotor specifies the left back motor of the drive base.
     * @param rfMotor specifies the right front motor of the drive base.
     * @param rcMotor specifies the right center motor of a 6-wheel drive base.
     * @param rbMotor specifies the right back motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcSimpleDriveBase(
        TrcMotor lfMotor, TrcMotor lcMotor, TrcMotor lbMotor, TrcMotor rfMotor, TrcMotor rcMotor, TrcMotor rbMotor,
        TrcGyro gyro)
    {
        super(new TrcMotor[] {lfMotor, rfMotor, lbMotor, rbMotor, lcMotor, rcMotor}, gyro);

        if (lfMotor == null || rfMotor == null ||
            lbMotor == null || rbMotor == null ||
            lcMotor == null || rcMotor == null)
        {
            throw new IllegalArgumentException("All 6 motors must not be null.");
        }

        this.lfMotor = lfMotor;
        this.rfMotor = rfMotor;
        this.lbMotor = lbMotor;
        this.rbMotor = rbMotor;
        this.lcMotor = lcMotor;
        this.rcMotor = rcMotor;
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 6-wheel drive base.
     *
     * @param lfMotor specifies the left front motor of the drive base.
     * @param lcMotor specifies the left center motor of a 6-wheel drive base.
     * @param lbMotor specifies the left back motor of the drive base.
     * @param rfMotor specifies the right front motor of the drive base.
     * @param rcMotor specifies the right center motor of a 6-wheel drive base.
     * @param rbMotor specifies the right back motor of the drive base.
     */
    public TrcSimpleDriveBase(
        TrcMotor lfMotor, TrcMotor lcMotor, TrcMotor lbMotor, TrcMotor rfMotor, TrcMotor rcMotor, TrcMotor rbMotor)
    {
        this(lfMotor, lcMotor, lbMotor, rfMotor, rcMotor, rbMotor, null);
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 4-wheel drive base.
     *
     * @param lfMotor specifies the left front motor of the drive base.
     * @param lbMotor specifies the left back motor of the drive base.
     * @param rfMotor specifies the right front motor of the drive base.
     * @param rbMotor specifies the right back motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcSimpleDriveBase(
        TrcMotor lfMotor, TrcMotor lbMotor, TrcMotor rfMotor, TrcMotor rbMotor, TrcGyro gyro)
    {
        super(new TrcMotor[] {lfMotor, rfMotor, lbMotor, rbMotor}, gyro);

        if (lfMotor == null || rfMotor == null || lbMotor == null || rbMotor == null)
        {
            throw new IllegalArgumentException("All 4 motors must not be null.");
        }

        this.lfMotor = lfMotor;
        this.rfMotor = rfMotor;
        this.lbMotor = lbMotor;
        this.rbMotor = rbMotor;
        this.lcMotor = null;
        this.rcMotor = null;
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 4-wheel drive base.
     *
     * @param lfMotor specifies the left front motor of the drive base.
     * @param lbMotor specifies the left back motor of the drive base.
     * @param rfMotor specifies the right front motor of the drive base.
     * @param rbMotor specifies the right back motor of the drive base.
     */
    public TrcSimpleDriveBase(
        TrcMotor lfMotor, TrcMotor lbMotor, TrcMotor rfMotor, TrcMotor rbMotor)
    {
        this(lfMotor, lbMotor, rfMotor, rbMotor, null);
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 2-wheel drive base.
     *
     * @param leftMotor specifies the left motor of the drive base.
     * @param rightMotor specifies the right motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcSimpleDriveBase(TrcMotor leftMotor, TrcMotor rightMotor, TrcGyro gyro)
    {
        super(new TrcMotor[] {leftMotor, rightMotor}, gyro);

        if (leftMotor == null || rightMotor == null)
        {
            throw new IllegalArgumentException("All 2 motors must not be null.");
        }

        this.lfMotor = leftMotor;
        this.rfMotor = rightMotor;
        this.lbMotor = null;
        this.rbMotor = null;
        this.lcMotor = null;
        this.rcMotor = null;
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 2-wheel drive base.
     *
     * @param leftMotor specifies the left motor of the drive base.
     * @param rightMotor specifies the right motor of the drive base.
     */
    public TrcSimpleDriveBase(TrcMotor leftMotor, TrcMotor rightMotor)
    {
        this(leftMotor, rightMotor, null);
    }   //TrcSimpleDriveBase

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
     * @param motorType specifies the motor in the drive train.
     * @param inverted specifies true if inverting motor direction.
     */
    public void setInvertedMotor(MotorType motorType, boolean inverted)
    {
        setInvertedMotor(motorType.value, inverted);
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

            if (lfMotor != null)
            {
                wheelPower = leftPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, lfMotor.getVelocity());
                }
                lfMotor.setPower(wheelPower);
            }

            if (rfMotor != null)
            {
                wheelPower = rightPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rfMotor.getVelocity());
                }
                rfMotor.setPower(wheelPower);
            }

            if (lbMotor != null)
            {
                wheelPower = leftPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, lbMotor.getVelocity());
                }
                lbMotor.setPower(wheelPower);
            }

            if (rbMotor != null)
            {
                wheelPower = rightPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rbMotor.getVelocity());
                }
                rbMotor.setPower(wheelPower);
            }

            if (lcMotor != null)
            {
                wheelPower = leftPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, lcMotor.getVelocity());
                }
                lcMotor.setPower(wheelPower);
            }

            if (rcMotor != null)
            {
                wheelPower = rightPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rcMotor.getVelocity());
                }
                rcMotor.setPower(wheelPower);
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
        Odometry delta = new Odometry(null, null);

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

        delta.position = new TrcPose2D(0.0, (lPos + rPos)/2 * yScale, (lPos - rPos)/2 * angleScale);
        delta.velocity = new TrcPose2D(0.0, (lVel + rVel)/2 * yScale, (lVel - rVel)/2 * angleScale);

        if (Math.abs(delta.velocity.y) > stallVelThreshold)
        {
            // reset stall start time to current time if drive base has movement.
            stallStartTime = TrcTimer.getCurrentTime();
        }

        return delta;
    }   //getOdometryDelta

}   //class TrcSimpleDriveBase
