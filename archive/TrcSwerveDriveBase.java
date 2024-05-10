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

package trclib.archive;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import java.util.Arrays;

/**
 * This class implements a platform independent swerve drive base. A swerve drive base consists of 4 swerve modules
 * each of which consists of a driving motor and a PID controlled steering motor. It extends the TrcSimpleDriveBase
 * class so it inherits all the SimpleDriveBase methods and features
 * <p>
 * The implementation of swerve algorithm is based on
 * <a href="https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383">
 * Ether's white paper</a>
 */
public class TrcSwerveDriveBase extends TrcSimpleDriveBase
{
    private static final String moduleName = TrcSwerveDriveBase.class.getSimpleName();

    private final TrcSwerveModule lfModule, rfModule, lbModule, rbModule;
    private final double wheelBaseWidth, wheelBaseLength, wheelBaseDiagonal;
    private final TrcHashMap<TrcMotor, TrcSwerveModule> driveMotorToModuleMap = new TrcHashMap<>();

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param lfModule specifies the left front swerve module of the drive base.
     * @param lbModule specifies the left back swerve module of the drive base.
     * @param rfModule specifies the right front swerve module of the drive base.
     * @param rbModule specifies the right back swerve module of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     * @param wheelBaseWidth specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     */
    public TrcSwerveDriveBase(
        TrcSwerveModule lfModule, TrcSwerveModule lbModule, TrcSwerveModule rfModule, TrcSwerveModule rbModule,
        TrcGyro gyro, double wheelBaseWidth, double wheelBaseLength)
    {
        super(lfModule.driveMotor, lbModule.driveMotor, rfModule.driveMotor, rbModule.driveMotor, gyro);

        this.lfModule = lfModule;
        this.rfModule = rfModule;
        this.lbModule = lbModule;
        this.rbModule = rbModule;
        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseLength = wheelBaseLength;
        this.wheelBaseDiagonal = TrcUtil.magnitude(wheelBaseWidth, wheelBaseLength);
        driveMotorToModuleMap.add(lfModule.driveMotor, lfModule);
        driveMotorToModuleMap.add(rfModule.driveMotor, rfModule);
        driveMotorToModuleMap.add(lbModule.driveMotor, lbModule);
        driveMotorToModuleMap.add(rbModule.driveMotor, rbModule);
    }   //TrcSwerveDriveBase

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param lfModule specifies the left front swerve module of the drive base.
     * @param lbModule specifies the left back swerve module of the drive base.
     * @param rfModule specifies the right front swerve module of the drive base.
     * @param rbModule specifies the right back swerve module of the drive base.
     * @param wheelBaseWidth  specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     */
    public TrcSwerveDriveBase(
        TrcSwerveModule lfModule, TrcSwerveModule lbModule, TrcSwerveModule rfModule, TrcSwerveModule rbModule,
        double wheelBaseWidth, double wheelBaseLength)
    {
        this(lfModule, lbModule, rfModule, rbModule, null, wheelBaseWidth, wheelBaseLength);
    }   //TrcSwerveDriveBase

    /**
     * This method does zero calibration on the steer angle encoders.
     */
    public void zeroCalibrateSteering()
    {
        lfModule.zeroCalibrateSteering();
        rfModule.zeroCalibrateSteering();
        lbModule.zeroCalibrateSteering();
        rbModule.zeroCalibrateSteering();
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
            lfModule.setSteerAngle(angle, optimize);
            rfModule.setSteerAngle(angle, optimize);
            lbModule.setSteerAngle(angle, optimize);
            rbModule.setSteerAngle(angle, optimize);
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

        TrcSwerveModule[] modules = {lfModule, rfModule, lbModule, rbModule};
        for (int i = 0; i < velocities.length; i++)
        {
            // Set angles before speed so angle optimization takes effect
            modules[i].setSteerAngle(velocities[i][1]);
            modules[i].driveMotor.setVelocity(velocities[i][0]);
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
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain for field relative drive. DO NOT use this with inverted.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    @Override
    protected void holonomicDrive(
        String owner, double x, double y, double rotation, boolean inverted, double gyroAngle, double driveTime,
        TrcEvent event)
    {
        tracer.traceDebug(
            moduleName,
            "owner=" + owner +
            ", x=" + x +
            ", y=" + y +
            ", rot=" + rotation +
            ", inverted=" + inverted +
            ", gyroAngle=" + gyroAngle +
            ", driveTime=" + driveTime +
            ", event=" + event);
        if (validateOwnership(owner))
        {
            if (x == 0.0 && y == 0.0 && rotation == 0.0)
            {
                lfModule.driveMotor.setPower(0.0);
                rfModule.driveMotor.setPower(0.0);
                lbModule.driveMotor.setPower(0.0);
                rbModule.driveMotor.setPower(0.0);

                // lfModule.setSteerAngle(lfModule.getSteerAngle());
                // rfModule.setSteerAngle(rfModule.getSteerAngle());
                // lbModule.setSteerAngle(lbModule.getSteerAngle());
                // rbModule.setSteerAngle(rbModule.getSteerAngle());
            }
            else
            {
                x = TrcUtil.clipRange(x);
                y = TrcUtil.clipRange(y);
                rotation = TrcUtil.clipRange(rotation);

                if (inverted)
                {
                    x = -x;
                    y = -y;
                }

                if (gyroAngle != 0)
                {
                    if (inverted)
                    {
                        tracer.traceWarn(
                            moduleName, "You should not be using inverted and field reference frame at the same time!");
                    }

                    double gyroRadians = Math.toRadians(gyroAngle);
                    double temp = y * Math.cos(gyroRadians) + x * Math.sin(gyroRadians);
                    x = -y * Math.sin(gyroRadians) + x * Math.cos(gyroRadians);
                    y = temp;
                }
                else if (isGyroAssistEnabled())
                {
                    rotation += getGyroAssistPower(rotation);
                }

                if (isAntiTippingEnabled())
                {
                    x += getAntiTippingPower(true);
                    y += getAntiTippingPower(false);
                }

                double rotLr = rotation * wheelBaseLength / wheelBaseDiagonal;
                double rotWr = rotation * wheelBaseWidth/ wheelBaseDiagonal;
                double a = x - rotLr;
                double b = x + rotLr;
                double c = y - rotWr;
                double d = y + rotWr;

                // The white paper goes in order rf, lf, lb, rb. We like to do lf, rf, lb, rb.
                // Note: atan2(y, x) in java will take care of x being zero.
                //       It will return pi/2 for positive y and -pi/2 for negative y.
                double[] steerAngles = new double[4];
                steerAngles[0] = Math.toDegrees(Math.atan2(b, d));
                steerAngles[1] = Math.toDegrees(Math.atan2(b, c));
                steerAngles[2] = Math.toDegrees(Math.atan2(a, d));
                steerAngles[3] = Math.toDegrees(Math.atan2(a, c));

                // The white paper goes in order rf, lf, lb, rb. We like to do lf, rf, lb, rb.
                double[] drivePowers = new double[4];
                drivePowers[0] = TrcUtil.magnitude(b, d);
                drivePowers[1] = TrcUtil.magnitude(b, c);
                drivePowers[2] = TrcUtil.magnitude(a, d);
                drivePowers[3] = TrcUtil.magnitude(a, c);

                TrcUtil.normalizeInPlace(drivePowers);
                if (motorPowerMapper != null)
                {
                    drivePowers[0] = motorPowerMapper.translateMotorPower(
                        drivePowers[0], lfModule.driveMotor.getVelocity());
                    drivePowers[1] = motorPowerMapper.translateMotorPower(
                        drivePowers[1], rfModule.driveMotor.getVelocity());
                    drivePowers[2] = motorPowerMapper.translateMotorPower(
                        drivePowers[2], lbModule.driveMotor.getVelocity());
                    drivePowers[3] = motorPowerMapper.translateMotorPower(
                        drivePowers[3], rbModule.driveMotor.getVelocity());
                }

                lfModule.setSteerAngle(steerAngles[0]);
                rfModule.setSteerAngle(steerAngles[1]);
                lbModule.setSteerAngle(steerAngles[2]);
                rbModule.setSteerAngle(steerAngles[3]);
                tracer.traceDebug(
                    moduleName,
                    "lfAngle=" + steerAngles[0] + "/" + (lfModule.getSteerAngle() % 360.0) +
                    ", rfAngle=" + steerAngles[1] + "/" + (rfModule.getSteerAngle() % 360.0) +
                    ", lbAngle=" + steerAngles[2] + "/" + (lbModule.getSteerAngle() % 360.0) +
                    ", rbAngle=" + steerAngles[3] + "/" + (rbModule.getSteerAngle() % 360.0));

                lfModule.setPower(drivePowers[0]);
                rfModule.setPower(drivePowers[1]);
                lbModule.setPower(drivePowers[2]);
                rbModule.setPower(drivePowers[3]);
                tracer.traceDebug(
                    moduleName,
                    "lfPower=" + drivePowers[0] +
                    ", rfPower=" + drivePowers[1] +
                    ", lbPower=" + drivePowers[2] +
                    ", rbPower=" + drivePowers[3]);

                if (drivePowers[0] == 0.0 && drivePowers[1] == 0.0 && drivePowers[2] == 0.0 && drivePowers[3] == 0.0)
                {
                    // reset stall start time to zero if drive base is stopped.
                    stallStartTime = 0.0;
                }
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
            lfModule.driveMotor.setPower(0.0);
            rfModule.driveMotor.setPower(0.0);
            lbModule.driveMotor.setPower(0.0);
            rbModule.driveMotor.setPower(0.0);
            lfModule.setSteerAngle(-45.0);
            rfModule.setSteerAngle(45.0);
            lbModule.setSteerAngle(45.0);
            rbModule.setSteerAngle(-45.0);
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
    protected Odometry getOdometryDelta(TrcOdometrySensor.Odometry[] prevOdometries,
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
            TrcSwerveModule swerveModule = driveMotorToModuleMap.get(currOdometries[i].sensor);
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

}   //class TrcSwerveDriveBase
