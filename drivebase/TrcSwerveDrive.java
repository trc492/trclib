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

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import java.util.Arrays;

import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcGyro;
import trclib.dataprocessor.TrcHashMap;
import trclib.motor.TrcMotor;
import trclib.sensor.TrcOdometrySensor;
import trclib.dataprocessor.TrcUtil;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent swerve drive base. A swerve drive base consists of 4 swerve modules
 * each of which consists of a driving motor and a PID controlled steering motor. It extends the TrcSimpleDrive
 * class so it inherits all the SimpleDriveBase methods and features
 * <p>
 * The implementation of swerve algorithm is based on
 * <a href="https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383">
 * Ether's white paper</a>
 */
public class TrcSwerveDrive extends TrcSimpleDrive
{
    private final String moduleName = getClass().getSimpleName();

    /**
     * This class contains tunable parameters of the Swerve Drive.
     */
    public static class SwerveParams
    {
        public TrcMotor.PidParams steerMotorPidParams = null;

        /**
         * This method sets the steer motor PID parameters.
         *
         * @param pidParams specifies the steer motor PID parameters.
         * @return this object for chaining.
         */
        public SwerveParams setSteerMotorPidParams(TrcMotor.PidParams pidParams)
        {
            this.steerMotorPidParams = pidParams;
            return this;
        }   //setSteerMotorPidParams
    }   //class SwerveParams

    protected final TrcSwerveModule[] swerveModules;
    private final double wheelBaseWidth, wheelBaseLength, wheelBaseDiagonal;
    private final TrcHashMap<TrcMotor, TrcSwerveModule> driveMotorToModuleMap = new TrcHashMap<>();

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param gyro specifies the gyro. If none, it can be set to null.
     * @param wheelBaseWidth specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     * @param flModule specifies the front left swerve module of the drive base.
     * @param frModule specifies the front right swerve module of the drive base.
     * @param blModule specifies the back left swerve module of the drive base.
     * @param brModule specifies the back right swerve module of the drive base.
     */
    public TrcSwerveDrive(
        TrcGyro gyro, double wheelBaseWidth, double wheelBaseLength,
        TrcSwerveModule flModule, TrcSwerveModule frModule, TrcSwerveModule blModule, TrcSwerveModule brModule)
    {
        super(gyro, flModule.driveMotor, frModule.driveMotor, blModule.driveMotor, brModule.driveMotor);

        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseLength = wheelBaseLength;
        this.wheelBaseDiagonal = TrcUtil.magnitude(wheelBaseWidth, wheelBaseLength);
 
        swerveModules = new TrcSwerveModule[4];
        swerveModules[MotorIndex.FrontLeft.value] = flModule;
        swerveModules[MotorIndex.FrontRight.value] = frModule;
        swerveModules[MotorIndex.BackLeft.value] = blModule;
        swerveModules[MotorIndex.BackRight.value] = brModule;

        driveMotorToModuleMap.add(flModule.driveMotor, flModule);
        driveMotorToModuleMap.add(frModule.driveMotor, frModule);
        driveMotorToModuleMap.add(blModule.driveMotor, blModule);
        driveMotorToModuleMap.add(brModule.driveMotor, brModule);
    }   //TrcSwerveDrive

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param wheelBaseWidth  specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     * @param flModule specifies the front left swerve module of the drive base.
     * @param frModule specifies the front right swerve module of the drive base.
     * @param blModule specifies the back left swerve module of the drive base.
     * @param brModule specifies the back right swerve module of the drive base.
     */
    public TrcSwerveDrive(
        double wheelBaseWidth, double wheelBaseLength,
        TrcSwerveModule flModule, TrcSwerveModule frModule, TrcSwerveModule blModule, TrcSwerveModule brModule)
    {
        this(null, wheelBaseWidth, wheelBaseLength, flModule, frModule, blModule, brModule);
    }   //TrcSwerveDrive

    /**
     * This method does zero calibration on the steer angle encoders.
     */
    public void zeroCalibrateSteering()
    {
        swerveModules[MotorIndex.FrontLeft.value].zeroCalibrateSteering();
        swerveModules[MotorIndex.FrontRight.value].zeroCalibrateSteering();
        swerveModules[MotorIndex.BackLeft.value].zeroCalibrateSteering();
        swerveModules[MotorIndex.BackRight.value].zeroCalibrateSteering();
    }   //zeroCalibrateSteering

    /**
     * This method returns the wheel base width.
     *
     * @return wheel base width.
     */
    public double getWheelBaseWidth()
    {
        return wheelBaseWidth;
    }   //getWheelBaseWidth

    /**
     * This method returns the wheel base length.
     *
     * @return wheel base length.
     */
    public double getWheelBaseLength()
    {
        return wheelBaseLength;
    }   //getWheelBaseLength

    /**
     * This method checks if it supports holonomic drive.
     *
     * @return true if this drive base supports holonomic drive, false otherwise.
     */
    @Override
    public boolean supportsHolonomicDrive()
    {
        return true;
    }   //supportsHolonomicDrive

    /**
     * This method sets the odometry scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param xScale     specifies the X position scale.
     * @param yScale     specifies the Y position scale.
     * @param angleScale specifies the angle scale.
     */
    @Override
    public void setOdometryScales(double xScale, double yScale, double angleScale)
    {
        if (xScale != yScale)
        {
            throw new IllegalArgumentException("Swerve does not have different x and y scales!");
        }

        super.setOdometryScales(xScale, yScale, angleScale);
    }   //setOdometryScales

    /**
     * This method sets the odometry scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param scale specifies the position scale for each motor.
     */
    @Override
    public void setOdometryScales(double scale)
    {
        super.setOdometryScales(scale, scale, 1.0);
    }   //setOdometryScales

    /**
     * This method stops all drive motors.
     */
    protected void stopDriveMotors()
    {
        for (TrcSwerveModule module: swerveModules)
        {
            module.driveMotor.setPower(0.0);
        }
    }   //stopDriveMotors

    /**
     * This method sets the steering angle of all four wheels.
     *
     * @param owner    specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                 ownership aware.
     * @param angle    specifies the steering angle to be set.
     * @param optimize specifies true to optimize steering angle to be no greater than 90 degrees, false otherwise.
     */
    public void setSteerAngle(String owner, double angle, boolean optimize)
    {
        tracer.traceDebug(
            moduleName,
            "owner=" + owner +
            ", angle=" + angle +
            ", optimize=" + optimize);
        if (validateOwnership(owner))
        {
            swerveModules[MotorIndex.FrontLeft.value].setSteerAngle(angle, optimize);
            swerveModules[MotorIndex.FrontRight.value].setSteerAngle(angle, optimize);
            swerveModules[MotorIndex.BackLeft.value].setSteerAngle(angle, optimize);
            swerveModules[MotorIndex.FrontRight.value].setSteerAngle(angle, optimize);
        }
    }   //setSteerAngle

    /**
     * This method sets the steering angle of all four wheels.
     *
     * @param angle    specifies the steering angle to be set.
     * @param optimize specifies true to optimize steering angle to be no greater than 90 degrees, false otherwise.
     */
    public void setSteerAngle(double angle, boolean optimize)
    {
        setSteerAngle(null, angle, optimize);
    }   //setSteerAngle

    /**
     * This method sets the steering angle of all four wheels.
     *
     * @param angle specifies the steering angle to be set.
     */
    public void setSteerAngle(double angle)
    {
        setSteerAngle(null, angle, true);
    }   //setSteerAngle

    /**
     * Set the velocities of the swerve modules.
     *
     * @param velocities 2d array. First dimension is number of modules, in order [lf, rf, lr, rr]. Next dimension is
     *                   polar vector in (r, theta). Theta is degrees CW, r is in range [-1,1].
     */
    public void setModuleVelocities(double[][] velocities)
    {
        if (velocities.length != getNumMotors())
        {
            throw new IllegalArgumentException("Invalid velocities parameter: " + Arrays.deepToString(velocities));
        }

        for (int i = 0; i < swerveModules.length; i++)
        {
            // Set angles before speed so angle optimization takes effect
            swerveModules[i].setSteerAngle(velocities[i][1]);
            swerveModules[i].driveMotor.setVelocity(velocities[i][0]);
        }
    }   //setModuleVelocities

    /**
     * This method stops the drive base. If steerNeutral is true, it also sets all steering angles to zero.
     *
     * @param owner      specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                   ownership aware.
     * @param resetSteer specifies true to set steering angle to zero.
     */
    public void stop(String owner, boolean resetSteer)
    {
        tracer.traceDebug(moduleName, "owner=" + owner + ", resetSteer=" + resetSteer);
        if (validateOwnership(owner))
        {
            super.stop(owner);

            if (resetSteer)
            {
                setSteerAngle(0.0, false);
            }
        }
    }   //stop

    /**
     * This method stops the drive base. If steerNeutral is true, it also sets all steering angles to zero.
     *
     * @param resetSteer specifies true to set steering angle to zero.
     */
    public void stop(boolean resetSteer)
    {
        stop(null, resetSteer);
    }   //stop

    /**
     * This method stops the drive base and reset the steering angle to zero.
     */
    @Override
    public void stop()
    {
        stop(null, false);
    }   //stop

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors. It will set the steering angle to zero, but note that it will take time for the steering angles to
     * reach zero. Since we do not wait for the steering angle to reach neutral, it is possible the drive base will
     * move diagonally initially. If this is undesirable, the caller should make sure steering angles are already at
     * zero before calling this method.
     *
     * @param owner      specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                   ownership aware.
     * @param leftPower  specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted   specifies true to invert control (i.e. robot front becomes robot back).
     * @param driveTime  specifies the amount of time in seconds after which the drive base will stop.
     * @param event      specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    @Override
    public void tankDrive(
        String owner, double leftPower, double rightPower, boolean inverted, double driveTime, TrcEvent event)
    {
        setSteerAngle(owner, 0.0, false);
        super.tankDrive(owner, leftPower, rightPower, inverted, driveTime, event);
    }   //tankDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     * <p>
     * The implementation of swerve algorithm is based on
     * <a href="https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf">
     * Ether's white paper</a>
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param xPower    specifies the x power.
     * @param yPower    specifies the y power.
     * @param turnPower specifies the rotating power.
     * @param gyroAngle specifies the gyro angle to maintain for field relative drive. DO NOT use this with inverted.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    @Override
    public void holonomicDrive(
        String owner, double xPower, double yPower, double turnPower, Double gyroAngle, double driveTime,
        TrcEvent event)
    {
        tracer.traceDebug(
            moduleName,
            "owner=" + owner +
            ", x=" + xPower +
            ", y=" + yPower +
            ", turn=" + turnPower +
            ", gyroAngle=" + gyroAngle +
            ", driveTime=" + driveTime +
            ", event=" + event);
        if (validateOwnership(owner))
        {
            if (xPower == 0.0 && yPower == 0.0 && turnPower == 0.0)
            {
                stopDriveMotors();
            }
            else
            {
                xPower = TrcUtil.clipRange(xPower);
                yPower = TrcUtil.clipRange(yPower);
                turnPower = TrcUtil.clipRange(turnPower);

                if (gyroAngle != null)
                {
                    double gyroRadians = Math.toRadians(gyroAngle);
                    double temp = yPower * Math.cos(gyroRadians) + xPower * Math.sin(gyroRadians);
                    xPower = -yPower * Math.sin(gyroRadians) + xPower * Math.cos(gyroRadians);
                    yPower = temp;
                }
                else if (isGyroAssistEnabled())
                {
                    turnPower += getGyroAssistPower(turnPower);
                }

                if (isAntiTippingEnabled())
                {
                    xPower += getAntiTippingPower(true);
                    yPower += getAntiTippingPower(false);
                }

                double rotLr = turnPower * wheelBaseLength / wheelBaseDiagonal;
                double rotWr = turnPower * wheelBaseWidth/ wheelBaseDiagonal;
                double a = xPower - rotLr;
                double b = xPower + rotLr;
                double c = yPower - rotWr;
                double d = yPower + rotWr;

                // The white paper goes in order rf, lf, lb, rb. We like to do lf, rf, lb, rb.
                // Note: atan2(y, x) in java will take care of x being zero.
                //       It will return pi/2 for positive y and -pi/2 for negative y.
                double[] steerAngles = new double[swerveModules.length];
                steerAngles[MotorIndex.FrontLeft.value] = Math.toDegrees(Math.atan2(b, d));
                steerAngles[MotorIndex.FrontRight.value] = Math.toDegrees(Math.atan2(b, c));
                steerAngles[MotorIndex.BackLeft.value] = Math.toDegrees(Math.atan2(a, d));
                steerAngles[MotorIndex.BackRight.value] = Math.toDegrees(Math.atan2(a, c));

                // The white paper goes in order rf, lf, lb, rb. We like to do lf, rf, lb, rb.
                double[] drivePowers = new double[swerveModules.length];
                drivePowers[MotorIndex.FrontLeft.value] = TrcUtil.magnitude(b, d);
                drivePowers[MotorIndex.FrontRight.value] = TrcUtil.magnitude(b, c);
                drivePowers[MotorIndex.BackLeft.value] = TrcUtil.magnitude(a, d);
                drivePowers[MotorIndex.BackRight.value] = TrcUtil.magnitude(a, c);

                TrcUtil.normalizeInPlace(drivePowers);
                if (motorPowerMapper != null)
                {
                    for (int i = 0; i < swerveModules.length; i++)
                    {
                        drivePowers[i] = motorPowerMapper.translateMotorPower(
                            drivePowers[i], swerveModules[i].driveMotor.getVelocity());
                    }
                }

                for (int i = 0; i < swerveModules.length; i++)
                {
                    swerveModules[i].setSteerAngle(steerAngles[i]);
                }
                tracer.traceDebug(
                    moduleName,
                    "flAngle=" + steerAngles[MotorIndex.FrontLeft.value] +
                    "/" + (swerveModules[MotorIndex.FrontLeft.value].getSteerAngle() % 360.0) +
                    ", frAngle=" + steerAngles[MotorIndex.FrontRight.value] +
                    "/" + (swerveModules[MotorIndex.FrontRight.value].getSteerAngle() % 360.0) +
                    ", blAngle=" + steerAngles[MotorIndex.BackLeft.value] +
                    "/" + (swerveModules[MotorIndex.BackLeft.value].getSteerAngle() % 360.0) +
                    ", brAngle=" + steerAngles[MotorIndex.BackRight.value] +
                    "/" + (swerveModules[MotorIndex.BackRight.value].getSteerAngle() % 360.0));

                boolean allStopped = true;
                for (int i = 0; i < swerveModules.length; i++)
                {
                    swerveModules[i].setPower(drivePowers[i]);
                    if (drivePowers[i] != 0.0)
                    {
                        allStopped = false;
                    }
                }

                if (allStopped)
                {
                    // reset stall start time to zero if drive base is stopped.
                    stallStartTime = 0.0;
                }
                tracer.traceDebug(
                    moduleName,
                    "flPower=" + drivePowers[MotorIndex.FrontLeft.value] +
                    ", frPower=" + drivePowers[MotorIndex.FrontRight.value] +
                    ", blPower=" + drivePowers[MotorIndex.BackLeft.value] +
                    ", brPower=" + drivePowers[MotorIndex.FrontRight.value]);
            }
            setDriveTime(owner, driveTime, event);
        }
    }   //holonomicDrive

    /**
     * This method set all the wheels into an X configuration so that nobody can bump us out of position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     */
    public void setXMode(String owner)
    {
        if (validateOwnership(owner))
        {
            stopDriveMotors();
            swerveModules[MotorIndex.FrontLeft.value].setSteerAngle(-45.0);
            swerveModules[MotorIndex.FrontRight.value].setSteerAngle(45.0);
            swerveModules[MotorIndex.BackLeft.value].setSteerAngle(45.0);
            swerveModules[MotorIndex.BackRight.value].setSteerAngle(-45.0);
        }
    }   //setXMode

    /**
     * This method is called periodically to calculate the delta between the previous and current motor odometries.
     *
     * @param prevOdometries specifies the previous motor odometries.
     * @param currOdometries specifies the current motor odometries.
     * @return an Odometry object describing the odometry changes since the last update.
     */
    @Override
    protected Odometry getOdometryDelta(
        TrcOdometrySensor.Odometry[] prevOdometries,
        TrcOdometrySensor.Odometry[] currOdometries)
    {
        Odometry delta = new Odometry();

        //
        // Average the posDelta vectors and velocity vectors of all four wheels:
        //  (sum posDelta vectors of all wheels)/num_of_wheels
        //  (sum velocity vectors of all wheels)/num_of_wheels
        //
        int numMotors = currOdometries.length;
        RealVector[] wheelPosVectors = new RealVector[numMotors];
        RealVector[] wheelVelVectors = new RealVector[numMotors];
        RealVector posSum = new ArrayRealVector(2);
        RealVector velSum = new ArrayRealVector(2);
        for (int i = 0; i < numMotors; i++)
        {
            TrcSwerveModule swerveModule = driveMotorToModuleMap.get((TrcMotor) currOdometries[i].sensor);
            // swerveModule won't be null but checking it to shut up the compiler warning.
            double angle = swerveModule != null? swerveModule.getSteerAngle(): 0.0;
            double posDelta = currOdometries[i].currPos - prevOdometries[i].currPos;
            // xScale and yScale on SwerveDrive should be identical.
            wheelPosVectors[i] = TrcUtil.polarToCartesian(posDelta, angle).mapMultiply(xScale);
            wheelVelVectors[i] = TrcUtil.polarToCartesian(currOdometries[i].velocity, angle).mapMultiply(xScale);
            posSum = posSum.add(wheelPosVectors[i]);
            velSum = velSum.add(wheelVelVectors[i]);
        }
        double multiplier = 1.0 / numMotors;
        posSum.mapMultiplyToSelf(multiplier);
        velSum.mapMultiplyToSelf(multiplier);
        //
        // Calculate the odometry delta.
        //
        delta.position.x = posSum.getEntry(0);
        delta.position.y = posSum.getEntry(1);

        delta.velocity.x = velSum.getEntry(0);
        delta.velocity.y = velSum.getEntry(1);

        if (TrcUtil.magnitude(delta.velocity.x, delta.velocity.y) > stallVelThreshold)
        {
            // reset stall start time to current time if drive base has movement.
            stallStartTime = TrcTimer.getCurrentTime();
        }

        double x = wheelBaseWidth / 2;
        double y = wheelBaseLength / 2;
        // This is black magic math, and it actually needs to be tested.
        // CodeReview: Please put a reference to your research material so we know where it came from.
        double dRot = x * (wheelPosVectors[0].getEntry(1) + wheelPosVectors[2].getEntry(1) -
                           wheelPosVectors[1].getEntry(1) - wheelPosVectors[3].getEntry(1)) +
                      y * (wheelPosVectors[0].getEntry(0) + wheelPosVectors[1].getEntry(0) -
                           wheelPosVectors[2].getEntry(0) - wheelPosVectors[3].getEntry(0));

        dRot /= 4 * Math.pow(wheelBaseDiagonal, 2);
        dRot = Math.toDegrees(dRot);
        delta.position.angle = dRot;

        double rotVel = x * (wheelVelVectors[0].getEntry(1) + wheelVelVectors[2].getEntry(1) -
                             wheelVelVectors[1].getEntry(1) - wheelVelVectors[3].getEntry(1)) +
                        y * (wheelVelVectors[0].getEntry(0) + wheelVelVectors[1].getEntry(0) -
                             wheelVelVectors[2].getEntry(0) - wheelVelVectors[3].getEntry(0));
        rotVel /= 4 * Math.pow(wheelBaseDiagonal, 2);
        rotVel = Math.toDegrees(rotVel);
        delta.velocity.angle = rotVel;

        return delta;
    }   //getOdometryDelta

}   //class TrcSwerveDrive
