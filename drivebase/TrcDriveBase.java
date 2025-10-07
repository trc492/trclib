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

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.Arrays;
import java.util.Stack;
import java.util.function.DoubleSupplier;

import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.dataprocessor.TrcWrapValueConverter;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.sensor.TrcDriveBaseOdometry;
import trclib.sensor.TrcGyro;
import trclib.sensor.TrcOdometrySensor;
import trclib.sensor.TrcOdometryWheels;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent drive base. It is intended to be extended by subclasses that
 * implements different drive base configurations (e.g. SimpleDriveBase, MecanumDriveBase and SwerveDriveBase).
 * The subclasses must provide the tankDrive method and optionally overriding holonomicDrive method if it support it.
 * If the subclass does support holonomic drive, it should override the supportsHolonomicDrive method. It must also
 * provide the getOdometryDelta method where it will calculate the drive base position and velocity info according to
 * sensors such as encoders and gyro.
 */
public abstract class TrcDriveBase implements TrcExclusiveSubsystem
{
    private final String moduleName = getClass().getSimpleName();

    /**
     * This class contains tunable parameters of the DriveBase.
     */
    public static class TuneParams
    {
        public double driveMotorMaxVelocity = 0.0;
        public TrcPidController.PidCoefficients driveMotorVelPidCoeffs = null;
        public double driveMotorVelPidTolerance = 0.0;
        public boolean driveMotorSoftwarePid = false;

        public double drivePidTolerance = 0.0;
        public double turnPidTolerance = 0.0;
        public double driveTime = 0.0;
        public TrcPidController.PidCoefficients xDrivePidCoeffs = null;
        public double xDriveTarget = 0.0;
        public double xDrivePowerLimit = 1.0;
        public TrcPidController.PidCoefficients yDrivePidCoeffs = null;
        public double yDriveTarget = 0.0;
        public double yDrivePowerLimit = 1.0;
        public TrcPidController.PidCoefficients turnPidCoeffs = null;
        public double turnTarget = 0.0;
        public double turnPowerLimit = 1.0;
        public TrcPidController.PidCoefficients velPidCoeffs = null;
//        public boolean enableSquid = false;

        public double profiledMaxDriveVelocity = 0.0;
        public double profiledMaxDriveAcceleration = 0.0;
        public double profiledMaxDriveDeceleration = 0.0;
        public double profiledMaxTurnRate = 0.0;

        /**
         * This method sets parameters for drive motor velocity control.
         *
         * @param maxVelocity specifies the max motor velocity.
         * @param velPidCoeffs specifies the velocity control PID Coefficients.
         * @param pidTolerance specifies the velocity PID tolerance.
         * @param useSoftwarePid specifies true to use software PID control, false to use motor native PID control.
         * @return this object for chaining.
         */
        public TuneParams setDriveMotorVelocityControl(
            double maxVelocity, TrcPidController.PidCoefficients velPidCoeffs, double pidTolerance,
            boolean useSoftwarePid)
        {
            this.driveMotorMaxVelocity = maxVelocity;
            this.driveMotorVelPidCoeffs = velPidCoeffs;
            this.driveMotorVelPidTolerance = pidTolerance;
            this.driveMotorSoftwarePid = useSoftwarePid;
            return this;
        }   //setDriveMotorVelocityControl

        /**
         * This method sets drive and turn tolerances of the drive base.
         *
         * @param driveTolerance specifies the drive tolerance in scaled unit.
         * @param turnTolerance specifies the turn tolerance in scaled unit.
         * @return this object for chaining.
         */
        public TuneParams setPidTolerances(double driveTolerance, double turnTolerance)
        {
            this.drivePidTolerance = driveTolerance;
            this.turnPidTolerance = turnTolerance;
            return this;
        }   //setPidTolerances

        /**
         * This method sets the parameters for drive base PID control in the X direction.
         *
         * @param pidCoeffs specifies the PID Coefficients.
         * @param powerLimit specifies the PID power limit.
         * @return this object for chaining.
         */
        public TuneParams setXPidParams(TrcPidController.PidCoefficients pidCoeffs, double powerLimit)
        {
            this.xDrivePidCoeffs = pidCoeffs;
            this.xDrivePowerLimit = powerLimit;
            return this;
        }   //setXPidParams

        /**
         * This method sets the parameters for drive base PID control in the Y direction.
         *
         * @param pidCoeffs specifies the PID Coefficients.
         * @param powerLimit specifies the PID power limit.
         * @return this object for chaining.
         */
        public TuneParams setYPidParams(TrcPidController.PidCoefficients pidCoeffs, double powerLimit)
        {
            this.yDrivePidCoeffs = pidCoeffs;
            this.yDrivePowerLimit = powerLimit;
            return this;
        }   //setYPidParams

        /**
         * This method sets the parameters for drive base PID control for turning.
         *
         * @param pidCoeffs specifies the PID Coefficients.
         * @param powerLimit specifies the PID power limit.
         * @return this object for chaining.
         */
        public TuneParams setTurnPidParams(TrcPidController.PidCoefficients pidCoeffs, double powerLimit)
        {
            this.turnPidCoeffs = pidCoeffs;
            this.turnPowerLimit = powerLimit;
            return this;
        }   //setTurnPidParams

        /**
         * This method sets the parameters for drive base velocity PID control. It is mainly used in PurePursuit drive.
         *
         * @param pidCoeffs specifies the PID Coefficients.
         * @return this object for chaining.
         */
        public TuneParams setVelocityPidParams(TrcPidController.PidCoefficients pidCoeffs)
        {
            this.velPidCoeffs = pidCoeffs;
            return this;
        }   //setVelocityPidParams

        /**
         * This method sets the characteristics of the drive base. This is used in PurePursuit drive for motion
         * profile drive.
         *
         * @param maxVelocity specifies the drive base maximum profiled velocity.
         * @param maxAcceleration specifies the drive base maximum profiled acceleration.
         * @param maxDeceleration specifies the drive base maximum profiled deceleration.
         * @param maxTurnRate specifies the drive base maximum turn rate.
         * @return this object for chaining.
         */
        public TuneParams setDriveCharacteristics(
            double maxVelocity, double maxAcceleration, double maxDeceleration, double maxTurnRate)
        {
            this.profiledMaxDriveVelocity = maxVelocity;
            this.profiledMaxDriveAcceleration = maxAcceleration;
            this.profiledMaxDriveDeceleration = maxDeceleration;
            this.profiledMaxTurnRate = maxTurnRate;
            return this;
        }   //setDriveCharacteristics
    }   //class TuneParams

    //
    // If true, the change in pose is a twist, and is applied to the current pose using a non-zero curvature
    // (non-zero rotation velocity).
    // If false, use zero curvature (assume path is a bunch of straight lines). This is less accurate.
    //
    private static final boolean USE_CURVED_PATH = true;
    private static final boolean SYNC_GYRO_DATA = false;

    /**
     * This enum specifies all the drive orientation modes:
     * - ROBOT: Robot centric driving mode.
     * - FIELD: Field centric driving mode.
     * - INVERTED: Inverted driving mode (i.e. robot front becomes robot rear and vice versa).
     */
    public enum DriveOrientation
    {
        ROBOT, FIELD, INVERTED;

        public static DriveOrientation nextDriveOrientation(DriveOrientation driveOrientation)
        {
            DriveOrientation nextDriveOrientation;

            switch (driveOrientation)
            {
                case ROBOT:
                    nextDriveOrientation = FIELD;
                    break;

                case FIELD:
                    nextDriveOrientation = INVERTED;
                    break;

                case INVERTED:
                default:
                    nextDriveOrientation = ROBOT;
                    break;
            }

            return nextDriveOrientation;
        }   //nextDriveOrientation

    }   //enum DriveOrientation

    /**
     * This specifies the type of Drive Base Odometry is in use.
     */
    public enum OdometryType
    {
        MotorOdometry,
        OdometryWheels,
        AbsoluteOdometry    //e.g. SparkFun OTOS or GoBilda Pinpoint Odometry
    }   //enum OdometryType

    /**
     * This class implements the drive base odometry. It consists of the position as well as velocity info in all
     * three degrees of movement (x, y, angle).
     */
    public static class Odometry
    {
        public TrcPose2D position;
        public TrcPose2D velocity;

        /**
         * Constructor: Create an instance of the object.
         */
        public Odometry()
        {
            position = new TrcPose2D();
            velocity = new TrcPose2D();
        }   //Odometry

        /**
         * Constructor: Create an instance of the object.
         *
         * @param position specifies the initial position.
         * @param velocity specifies the initial velocity.
         */
        public Odometry(TrcPose2D position, TrcPose2D velocity)
        {
            this.position = position;
            this.velocity = velocity;
        }   //Odometry

        /**
         * This method returns the string representation of the object.
         *
         * @return string representation of the object.
         */
        @Override
        public String toString()
        {
            return "(pos=" + position + ",vel=" + velocity + ")";
        }   //toString

        /**
         * This method creates and returns a copy of this odometry.
         *
         * @return a copy of this odometry.
         */
        @Override
        public Odometry clone()
        {
            return new Odometry(position.clone(), velocity.clone());
        }   //clone

        /**
         * This method sets the position info of the odometry to the given pose.
         *
         * @param pose specifies the pose to set the position info to.
         */
        void setPositionAs(TrcPose2D pose)
        {
            this.position.setAs(pose);
        }   //setPositionAs

        /**
         * This method sets the velocity info of the odometry to the given pose.
         *
         * @param pose specifies the pose to set the velocity info to.
         */
        void setVelocityAs(TrcPose2D pose)
        {
            this.velocity.setAs(pose);
        }   //setVelocityAs

    }   //class Odometry

    /**
     * This class stores the states of all motors of the drivebase. This is used for calculating the drive base
     * odometry.
     */
    protected static class MotorsState
    {
        TrcOdometrySensor.Odometry[] prevMotorOdometries;
        TrcOdometrySensor.Odometry[] currMotorOdometries;

        public String toString()
        {
            return "odometry" + Arrays.toString(currMotorOdometries);
        }   //toString

    }   //class MotorsState

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param owner      specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                   ownership aware.
     * @param leftPower  specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted   specifies true to invert control (i.e. robot front becomes robot back).
     * @param driveTime  specifies the amount of time in seconds after which the drive base will stop.
     * @param event      specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public abstract void tankDrive(
        String owner, double leftPower, double rightPower, boolean inverted, double driveTime, TrcEvent event);

    /**
     * This method is called periodically to calculate the delta between the previous and current motor odometries.
     *
     * @param prevOdometries specifies the previous motor odometries.
     * @param currOdometries specifies the current motor odometries.
     * @return an Odometry object describing the odometry changes since the last update.
     */
    protected abstract Odometry getOdometryDelta(
        TrcOdometrySensor.Odometry[] prevOdometries, TrcOdometrySensor.Odometry[] currOdometries);

    /**
     * This interface is provided by the caller to translate the motor power to actual motor power according to
     * the motor curve. This is useful to linearize the motor performance. This is very useful for many reasons.
     * It could allow the drive base to drive straight by translating wheel power to actual torque. It could also
     * allow us to implement our own ramp rate to limit acceleration and deceleration.
     */
    public interface MotorPowerMapper
    {
        /**
         * This method is called to translate the desired motor power to the actual motor power taking into
         * consideration of the motor torque curve with the current motor velocity.
         *
         * @param power    specifies the desired motor power.
         * @param velocity specifies the current motor velocity in the unit of encoder counts per second.
         * @return resulting motor power.
         */
        double translateMotorPower(double power, double velocity);
    }   //interface MotorPowerMapper

    private static final double DEF_SENSITIVITY = 0.5;

    public final TrcDbgTrace tracer;
    private final TrcMotor[] motors;
    private final TrcGyro gyro;
    protected final Odometry odometry;
    private final MotorsState motorsState;
    private final TrcTimer driveTimer;
    private TrcEvent driveTimerEvent = null;
    private final TrcTaskMgr.TaskObject odometryTaskObj;
    protected double xScale, yScale, angleScale;
    private final Stack<Odometry> referenceOdometryStack = new Stack<>();
    private DriveOrientation driveOrientation = DriveOrientation.ROBOT;
    private double fieldForwardHeading;

    private String driveOwner = null;
    protected double stallStartTime = 0.0;
    protected double stallVelThreshold = 0.0;
    private OdometryType odometryType = OdometryType.MotorOdometry;
    private TrcOdometryWheels odometryWheels = null;
    private TrcDriveBaseOdometry absoluteOdometry = null;
    private TrcWrapValueConverter absOdoHeadingWrapConverter = null;
    protected MotorPowerMapper motorPowerMapper = null;
    private double sensitivity = DEF_SENSITIVITY;

    // GyroAssist driving.
    private TrcPidController gyroAssistPidCtrl = null;
    private Double gyroAssistHeading = null;
    private boolean robotTurning = false;

    private TrcPidController xTippingPidCtrl = null;
    private TrcPidController yTippingPidCtrl = null;
    private double xTippingTolerance = 0.0;
    private double yTippingTolerance = 0.0;
    private boolean antiTippingEnabled = false;
    private Odometry referenceOdometry = null;
    private boolean synchronizeOdometries = false;
    // Change of basis matrices to convert between coordinate systems
    private final RealMatrix enuToNwuChangeOfBasis = MatrixUtils
        .createRealMatrix(new double[][] { { 0, 1 }, { -1, 0 } });
    private final RealMatrix nwuToEnuChangeOfBasis = enuToNwuChangeOfBasis.transpose();

    /**
     * Constructor: Create an instance of the object.
     *
     * @param motors specifies the array of motors in the drive base.
     * @param gyro   specifies the gyro. If none, it can be set to null.
     */
    public TrcDriveBase(TrcMotor[] motors, TrcGyro gyro)
    {
        this.tracer = new TrcDbgTrace();
        this.motors = motors;
        this.gyro = gyro;

        odometry = new Odometry();
        motorsState = new MotorsState();
        motorsState.prevMotorOdometries = new TrcOdometrySensor.Odometry[motors.length];
        motorsState.currMotorOdometries = new TrcOdometrySensor.Odometry[motors.length];
        for (int i = 0; i < motors.length; i++)
        {
            motorsState.prevMotorOdometries[i] = null;
            motorsState.currMotorOdometries[i] = new TrcOdometrySensor.Odometry(motors[i]);
        }
        resetOdometry(true, true, true);
        driveTimer = new TrcTimer(moduleName + ".driveTimer");

        odometryTaskObj = TrcTaskMgr.createTask(moduleName + ".odometryTask", this::odometryTask);
        TrcTaskMgr.TaskObject stopTaskObj = TrcTaskMgr.createTask(moduleName + ".stopTask", this::stopTask);
        stopTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);

        xScale = yScale = angleScale = 1.0;
        fieldForwardHeading = getHeading();
    }   //TrcDriveBase

    /**
     * Constructor: Create an instance of the object.
     *
     * @param motors specifies the array of motors in the drive base.
     */
    public TrcDriveBase(TrcMotor... motors)
    {
        this(motors, null);
    }   //TrcDriveBase

    /**
     * This method returns an array of motors on the drive base.
     *
     * @return motor array.
     */
    public TrcMotor[] getMotors()
    {
        return motors;
    }   //getMotors

    /**
     * This method is called by the subclass to set a drive time timer.
     *
     * @param owner specifies the owner's ID string to be used to stop the drivebase, can be null if drivebase is not
     *              owned.
     * @param driveTime specifies the drive time in seconds after which the drive base is stopped.
     * @param event      specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    protected void setDriveTime(String owner, double driveTime, TrcEvent event)
    {
        if (driveTime > 0.0)
        {
            driveTimer.cancel();
            driveOwner = owner;
            driveTimerEvent = event;
            driveTimer.set(driveTime, this::driveTimerHandler);
        }
    }   //setDriveTime

    /**
     * This method is called when the drivebase drive timer has expired. It will stop the drivebase.
     *
     * @param context specifies the timer object (not used).
     * @param canceled specifies true if timer is canceled.
     */
    private void driveTimerHandler(Object context, boolean canceled)
    {
        stop(driveOwner);
        if (driveTimerEvent != null)
        {
            if (canceled)
            {
                driveTimerEvent.cancel();
            }
            else
            {
                driveTimerEvent.signal();
            }
            driveTimerEvent = null;
        }
        driveOwner = null;
    }   //driveTimerHandler

    /**
     * This method resets the field forward heading to the robot's current heading. This is only applicable if the
     * robot is in FIELD mode.
     */
    public void resetFieldForwardHeading()
    {
        if (driveOrientation == DriveOrientation.FIELD)
        {
            fieldForwardHeading = getHeading();
        }
    }   //resetFieldForwardHeading

    /**
     * This method sets the drive orientation mode.
     *
     * @param orientation specifies the drive orientation (FIELD, ROBOT, INVERTED).
     * @param resetHeading specifies true to reset robot heading, false otherwise (only applicable for FIELD mode).
     */
    public void setDriveOrientation(DriveOrientation orientation, boolean resetHeading)
    {
        // Don't allow setting FIELD mode if drive base doesn't support holonomic drive.
        if (orientation != DriveOrientation.FIELD || supportsHolonomicDrive())
        {
            driveOrientation = orientation;
            // If switching to FIELD oriented driving, reset robot heading so that the current robot heading is
            // "forward".
            if (resetHeading)
            {
                resetFieldForwardHeading();
            }
        }
        else
        {
            throw new UnsupportedOperationException("Drive base does not support holonomic drive.");
        }
    }   //setDriveOrientation

    /**
     * This method returns the current drive orientation mode.
     */
    public DriveOrientation getDriveOrientation()
    {
        return driveOrientation;
    }   //getDriveOrientation

    /**
     * This method returns robot heading to be maintained in teleop drive according to drive orientation mode.
     *
     * @return robot heading to be maintained.
     */
    public double getDriveGyroAngle()
    {
        double angle = 0.0;

        switch (driveOrientation)
        {
            case ROBOT:
                angle = 0.0;
                break;

            case INVERTED:
                angle = 180.0;
                break;

            case FIELD:
                angle = getHeading() - fieldForwardHeading;
                break;
        }

        return angle;
    }   //getDriveGyroAngle

    /**
     * This method checks if synchronize odometries is enabled.
     *
     * @return true if it is enabled, false if it is disabled.
     */
    public boolean isSynchronizeOdometriesEnabled()
    {
        synchronized (odometry)
        {
            return synchronizeOdometries;
        }
    }   //isSynchronizeOdometriesEnabled

    /**
     * This method enables/disables synchronize odometries.
     *
     * @param synchronizeOdometries specifies true to enable, false to disable.
     */
    public void setSynchronizeOdometriesEnabled(boolean synchronizeOdometries)
    {
        synchronized (odometry)
        {
            this.synchronizeOdometries = synchronizeOdometries;
        }
    }   //setSynchronizeOdometriesEnabled

    /**
     * This method enables/disables drive motor odometry.
     *
     * @param enabled specifies true to enable drive motor odometry, false to disable.
     * @param resetHardware specifies true to also reset motor encoder when enabling, false otherwise.
     */
    private void setMotorOdometryEnabled(boolean enabled, boolean resetHardware)
    {
        if (odometryType == OdometryType.MotorOdometry)
        {
            for (TrcMotor motor : motors)
            {
                motor.setOdometryEnabled(enabled, enabled, resetHardware);
            }
        }
    }   //setMotorOdometryEnabled

    /**
     * This method is called to enable/disable the odometry task that keeps track of the robot position and orientation.
     *
     * @param enabled specifies true to enable, false to disable.
     * @param resetHardware specifies true to reset odometry hardware, false otherwise. This is only applicable when
     *        enabling odometry, not used when disabling.
     */
    public void setOdometryEnabled(boolean enabled, boolean resetHardware)
    {
        tracer.traceDebug(moduleName, "enabled=" + enabled + ",resetHW=" + resetHardware);
        if (enabled)
        {
            setMotorOdometryEnabled(true, resetHardware);
            // We already reset position odometry when enabling motor odometry, so don't do it twice.
            // But we do need to reset heading odometry.
            resetOdometry(false, true, resetHardware);
            odometryTaskObj.registerTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }
        else
        {
            odometryTaskObj.unregisterTask();
            setMotorOdometryEnabled(false, false);
        }
    }   //setOdometryEnabled

    /**
     * This method is called to enable/disable the odometry task that keeps track of the robot position and orientation.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setOdometryEnabled(boolean enabled)
    {
        setOdometryEnabled(enabled, false);
    }   //setOdometryEnabled

    /**
     * This method returns the robot position in reference to the field origin. By default, the field origin is the
     * robot's starting position.
     *
     * @return a copy of the robot position relative to the field origin.
     */
    public TrcPose2D getFieldPosition()
    {
        synchronized (odometry)
        {
            return odometry.position.clone();
        }
    }   //getFieldPosition

    /**
     * This method returns the robot velocity in reference to the field origin. By default, the field origin is the
     * robot's starting position.
     *
     * @return a copy of the robot velocity relative to the field origin.
     */
    public TrcPose2D getFieldVelocity()
    {
        synchronized (odometry)
        {
            return odometry.velocity.clone();
        }
    }   //getFieldVelocity

    /**
     * This method sets the robot's absolute field position to the given pose. This can be used to set the robot's
     * starting position relative to the field origin.
     *
     * @param pose specifies the absolute position of the robot relative to the field origin.
     * @param positionOnly specifies true for setting position only but not heading, false to also set heading.
     */
    public void setFieldPosition(TrcPose2D pose, boolean positionOnly)
    {
        synchronized (odometry)
        {
            if (positionOnly)
            {
                // Setting position only, so restore the current robot heading.
                pose.angle = getHeading();
            }
            resetOdometry();
            odometry.setPositionAs(pose);
            if (absoluteOdometry != null)
            {
                absoluteOdometry.setPosition(pose);
            }
        }
    }   //setFieldPosition

    /**
     * This method sets the robot's absolute field position to the given pose. This can be used to set the robot's
     * starting position relative to the field origin.
     *
     * @param pose specifies the absolute position of the robot relative to the field origin.
     */
    public void setFieldPosition(TrcPose2D pose)
    {
        setFieldPosition(pose, false);
    }   // setFieldPosition

    /**
     * This method returns the robot position relative to <code>pose</code>.
     *
     * @param posPose specifies the position to be referenced to.
     * @param transformAngle specifies true to also transform angle, false to leave it alone.
     * @return position transformed into the new reference pose.
     */
    public TrcPose2D getPositionRelativeTo(TrcPose2D posPose, boolean transformAngle)
    {
        synchronized (odometry)
        {
            return odometry.position.relativeTo(posPose, transformAngle);
        }
    }   //getPositionRelativeTo

    /**
     * This method returns the robot position relative to <code>pose</code>.
     *
     * @param posPose specifies the position to be referenced to.
     * @return position transformed into the new reference pose.
     */
    public TrcPose2D getPositionRelativeTo(TrcPose2D posPose)
    {
        return getPositionRelativeTo(posPose, true);
    }   //getPositionRelativeTo

    /**
     * This method returns the robot velocity relative to <code>pose</code>.
     *
     * @param velPose specifies the velocity to be referenced to.
     * @param refAngle specifies the reference angle to be relative to.
     * @return velocity transformed into the new reference pose.
     */
    public TrcPose2D getVelocityRelativeTo(TrcPose2D velPose, double refAngle)
    {
        synchronized (odometry)
        {
            //
            // relativeTo will transform the odometry velocity vector to be relative to the angle of pose but the
            // angle of velPose is really the angular velocity not an angle, so we must duplicate velPose to a new
            // pose and change the angle member to be the refAngle and let the caller provide that angle.
            //
            TrcPose2D pose = velPose.clone();
            pose.angle = refAngle;
            return odometry.velocity.relativeTo(pose, false);
        }
    }   //getVelocityRelativeTo

    /**
     * This method returns the robot position relative to the reference position, or the robot field position if
     * there is no reference odometry set.
     *
     * @return robot position relative to the reference position, or robot field position if no reference odometry
     *         set.
     */
    public TrcPose2D getRelativePosition()
    {
        synchronized (odometry)
        {
            return referenceOdometry == null ?
                    getFieldPosition() : getPositionRelativeTo(referenceOdometry.position, true);
        }
    }   //getRelativePosition

    /**
     * This method returns the robot velocity relative to the reference velocity, or the robot field velocity if
     * there is no reference odometry set.
     *
     * @return robot velocity relative to the reference velocity, or robot field velocity if no reference odometry
     *         set.
     */
    public TrcPose2D getRelativeVelocity()
    {
        synchronized (odometry)
        {
            return referenceOdometry == null ?
                    getFieldVelocity() :
                    getVelocityRelativeTo(referenceOdometry.velocity, referenceOdometry.position.angle);
        }
    }   //getRelativeVelocity

    /**
     * This method returns the reference odometry if there is any.
     *
     * @return the reference odometry.
     */
    public Odometry getReferenceOdometry()
    {
        synchronized (odometry)
        {
            return referenceOdometry;
        }
    }   //getReferenceOdometry

    /**
     * This method sets the current robot position and velocity as the reference odometry. All relative positions
     * and velocities will be relative to this reference odometry.
     */
    public void setReferenceOdometry()
    {
        synchronized (odometry)
        {
            referenceOdometry = odometry.clone();
        }
    }   //setReferenceOdometry

    /**
     * This method clears the reference odometry. All relative positions and velocities will instead be absolute
     * positions and velocities.
     */
    public void clearReferenceOdometry()
    {
        synchronized (odometry)
        {
            referenceOdometry = null;
        }
    }   //clearReferenceOdometry

    /**
     * This method pushes the existing reference odometry onto the stack if there is one and set the given robot's
     * absolute odometry as the new reference odometry.
     */
    public void pushReferenceOdometry()
    {
        synchronized (odometry)
        {
            if (referenceOdometry != null)
            {
                referenceOdometryStack.push(referenceOdometry);
            }

            setReferenceOdometry();
        }
    }   //pushReferenceOdometry

    /**
     * This method returns the current reference odometry. If the reference stack is not empty, it pops an odometry
     * from the stack as the new reference odometry, otherwise the reference odometry is cleared.
     *
     * @return current reference odometry.
     */
    public Odometry popReferenceOdometry()
    {
        synchronized (odometry)
        {
            Odometry oldReferenceOdometry = referenceOdometry;

            if (!referenceOdometryStack.empty())
            {
                referenceOdometry = referenceOdometryStack.pop();
            }
            else
            {
                referenceOdometry = null;
            }

            return oldReferenceOdometry;
        }
    }   //popReferenceOdometry

    /**
     * This method sets the position scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param xScale   specifies the X position scale.
     * @param yScale   specifies the Y position scale.
     * @param angleScale specifies the angle scale.
     */
    public void setOdometryScales(double xScale, double yScale, double angleScale)
    {
        synchronized (odometry)
        {
            this.xScale = xScale;
            this.yScale = yScale;
            this.angleScale = angleScale;

            if (odometryWheels != null)
            {
                odometryWheels.setOdometryScales(xScale, yScale, angleScale);
            }
        }
    }   //setOdometryScales

    /**
     * This method sets the position scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param xScale specifies the X position scale.
     * @param yScale specifies the Y position scale.
     */
    public void setOdometryScales(double xScale, double yScale)
    {
        setOdometryScales(xScale, yScale, 1.0);
    }   //setOdometryScales

    /**
     * This method sets the position scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param yScale specifies the Y position scale.
     */
    public void setOdometryScales(double yScale)
    {
        setOdometryScales(1.0, yScale, 1.0);
    }   //setOdometryScales

    /**
     * This method returns the X position in scaled unit.
     *
     * @return X position.
     */
    public double getXPosition()
    {
        return getRelativePosition().x;
    }   //getXPosition

    /**
     * This method returns the Y position in scaled unit.
     *
     * @return Y position.
     */
    public double getYPosition()
    {
        return getRelativePosition().y;
    }   //getYPosition

    /**
     * This method returns the heading of the drive base in degrees. If there is a gyro, the gyro heading is returned,
     * otherwise it returns the rotation position by using the encoders.
     *
     * @return drive base heading.
     */
    public double getHeading()
    {
        synchronized (odometry)
        {
            return odometry.position.angle;
        }
    }   //getHeading

    /**
     * This method returns the drive base velocity in the X direction.
     *
     * @return X velocity.
     */
    public double getXVelocity()
    {
        return getRelativeVelocity().x;
    }   //getXVelocity

    /**
     * This method returns the drive base velocity in the Y direction.
     *
     * @return Y velocity.
     */
    public double getYVelocity()
    {
        return getRelativeVelocity().y;
    }   //getYVelocity

    /**
     * This method returns the drive base turn rate.
     *
     * @return turn rate.
     */
    public double getTurnRate()
    {
        synchronized (odometry)
        {
            return odometry.velocity.angle;
        }
    }   //getGyroTurnRate

    /**
     * This method resets the drive base odometry. This includes the motor encoders, drive base position, velocity and
     * gyro heading.
     *
     * @param resetPositionOdometry  specifies true for resetting position odometry, false otherwise.
     * @param resetHeadingOdometry specifies true to also reset the heading odometry, false otherwise.
     * @param resetHardware specifies true to do a hardware reset, false to do a soft reset.
     */
    public void resetOdometry(boolean resetPositionOdometry, boolean resetHeadingOdometry, boolean resetHardware)
    {
        tracer.traceDebug(
            moduleName,
            "resetPosOd=" + resetPositionOdometry +
            ", resetHeadingOd=" + resetHeadingOdometry +
            ", resetHardware=" + resetHardware);
        synchronized (odometry)
        {
            clearReferenceOdometry();

            if (odometryWheels != null)
            {
                odometryWheels.resetOdometry(resetPositionOdometry, resetHeadingOdometry, resetHardware);
            }
            else if (absoluteOdometry != null)
            {
                absoluteOdometry.reset();
                if (absOdoHeadingWrapConverter != null)
                {
                    absOdoHeadingWrapConverter.resetConverter();
                }
            }
            else
            {
                for (int i = 0; i < motors.length; i++)
                {
                    if (resetPositionOdometry)
                    {
                        motors[i].resetOdometry(resetHardware);
                    }
                    motorsState.prevMotorOdometries[i] = null;
                    motorsState.currMotorOdometries[i].prevTimestamp
                        = motorsState.currMotorOdometries[i].currTimestamp
                        = TrcTimer.getCurrentTime();
                    motorsState.currMotorOdometries[i].prevPos
                        = motorsState.currMotorOdometries[i].currPos
                        = motorsState.currMotorOdometries[i].velocity = 0.0;
                }

                if (resetHeadingOdometry)
                {
                    if (gyro != null)
                    {
                        gyro.resetOdometry(resetHardware);
                    }
                    odometry.position.angle = odometry.velocity.angle = 0.0;
                }
            }

            odometry.position.x = odometry.position.y = 0.0;
            odometry.velocity.x = odometry.velocity.y = 0.0;
        }
    }   //resetOdometry

    /**
     * This method resets the drive base odometry. This includes the motor encoders, drive base position, velocity and
     * gyro heading.
     *
     * @param resetHardware specifies true for resetting hardware position, false for resetting software position.
     */
    public void resetOdometry(boolean resetHardware)
    {
        resetOdometry(true, true, resetHardware);
    }   //resetOdometry

    /**
     * This method resets the drive base odometry. This includes the motor encoders, drive base position, velocity and
     * gyro heading.
     */
    public void resetOdometry()
    {
        resetOdometry(true, true, false);
    }   //resetOdometry

    /**
     * This method sets the built-in motor odometry as the drive base's odometry device.
     *
     * @param resetHardware specifies true for resetting hardware position, false for resetting software position.
     */
    public void setDriveBaseOdometry(boolean resetHardware)
    {
        synchronized (odometry)
        {
            // Disabling motor odometry if it was active before.
            setMotorOdometryEnabled(true, resetHardware);
            this.odometryWheels = null;
            this.absoluteOdometry = null;
            this.absOdoHeadingWrapConverter = null;
            this.odometryType = OdometryType.MotorOdometry;
        }
    }   //setDriveBaseOdometry

    /**
     * This method sets the given odometry device as the drive base's odometry device overriding the built-in odometry.
     *
     * @param odometryWheels specifies the odometry wheels as the drive base odometry device.
     */
    public void setDriveBaseOdometry(TrcOdometryWheels odometryWheels)
    {
        synchronized (odometry)
        {
            // Disabling motor odometry if it was active before.
            setMotorOdometryEnabled(false, false);
            this.odometryWheels = odometryWheels;
            this.absoluteOdometry = null;
            this.absOdoHeadingWrapConverter = null;
            this.odometryType = OdometryType.OdometryWheels;
        }
    }   //setDriveBaseOdometry

    /**
     * This method sets the given odometry device as the drive base's odometry device overriding the built-in odometry.
     *
     * @param absoluteOdometry specifies the drive base absolute odometry device.
     * @param headingWrapRangeLow specifies the low value of the heading wrap range, can be null if no wrapping.
     * @param headingWrapRangeHigh specifies the high value of the heading wrap range, can be null if no wrapping.
     */
    public void setDriveBaseOdometry(
        TrcDriveBaseOdometry absoluteOdometry, Double headingWrapRangeLow, Double headingWrapRangeHigh)
    {
        synchronized (odometry)
        {
            // Disabling motor odometry if it was active before.
            setMotorOdometryEnabled(false, false);
            this.odometryWheels = null;
            this.absoluteOdometry = absoluteOdometry;
            if (headingWrapRangeLow != null && headingWrapRangeHigh != null)
            {
                absOdoHeadingWrapConverter = new TrcWrapValueConverter(
                    moduleName + ".absOdoHeadingWrap", ()->{return absoluteOdometry.getPosition().angle;},
                    headingWrapRangeLow, headingWrapRangeHigh);
                absOdoHeadingWrapConverter.setTaskEnabled(true);
            }
            else
            {
                absOdoHeadingWrapConverter = null;
            }
            this.odometryType = OdometryType.AbsoluteOdometry;
        }
    }   //setDriveBaseOdometry

    /**
     * This method is called to print the state info of all motors on the drive base for debugging purpose.
     */
    public void printMotorsState()
    {
        synchronized (odometry)
        {
            tracer.traceInfo(moduleName, "motorsState=" + motorsState);
        }
    }   //printMotorsState

    /**
     * This method sets a motor power mapper. If null, it unsets the previously set mapper.
     *
     * @param motorPowerMapper specifies the motor power mapper. If null, clears the mapper.
     */
    public void setMotorPowerMapper(MotorPowerMapper motorPowerMapper)
    {
        this.motorPowerMapper = motorPowerMapper;
    }   //setMotorPowerMapper

    /**
     * This method sets the sensitivity for the drive() method.
     *
     * @param sensitivity specifies the sensitivity value.
     */
    public void setSensitivity(double sensitivity)
    {
        this.sensitivity = sensitivity;
    }   //setSensitivity

    /**
     * This method enables/disables gyro assist drive.
     *
     * @param turnPidCtrl specifies the turn PID controller to enable GyroAssist, null to disable.
     */
    public void setGyroAssistEnabled(TrcPidController turnPidCtrl)
    {
        this.gyroAssistPidCtrl = turnPidCtrl;
    }   //setGyroAssistEnabled

    /**
     * This method checks if Gyro Assist is enabled.
     *
     * @return true if Gyro Assist is enabled, false otherwise.
     */
    public boolean isGyroAssistEnabled()
    {
        return gyroAssistPidCtrl != null;
    }   //isGyroAssistEnabled

    /**
     * This method calculates and returns the gyro assist power to be added to turnPower to compensate heading drift.
     *
     * @param turnPower specifies the turn power.
     * @return gyro assist power.
     */
    protected double getGyroAssistPower(double turnPower)
    {
        double gyroAssistPower = 0.0;

        if (gyroAssistPidCtrl != null)
        {
            if (turnPower != 0.0)
            {
                // Robot is turning, do not need to apply GyroAssist power.
                robotTurning = true;
            }
            else
            {
                // The robot is going straight.
                if (robotTurning || gyroAssistHeading == null)
                {
                    // Robot just stopped turning or this is the first call, save the current robot heading.
                    // Set the current robot heading as the PID target to maintain this heading.
                    gyroAssistHeading = getHeading();
                    gyroAssistPidCtrl.setTarget(gyroAssistHeading);
                    robotTurning = false;
                    tracer.traceDebug(moduleName, "Maintain robot heading at " + gyroAssistHeading);
                }
                // Robot is going straight, use turnPid controller to calculate GyroAssist power.
                gyroAssistPower = gyroAssistPidCtrl.calculate();
            }
        }

        return gyroAssistPower;
    }   //getGyroAssistPower

    /**
     * This method enables anti-tipping drive.
     *
     * @param xTippingPidCoeffs specifies the anti-tipping PID controller coefficients for the X direction, null for
     *        non-holonomic drive base.
     * @param xTolerance specifies the X tipping tolerance.
     * @param xPidInput specifies the X PidInput provider.
     * @param yTippingPidCoeffs specifies the anti-tipping PID controller coefficients for the Y direction.
     * @param yTolerance specifies the Y tipping tolerance.
     * @param yPidInput specifies the Y PidInput provider.
     */
    public void enableAntiTipping(
        TrcPidController.PidCoefficients xTippingPidCoeffs, double xTolerance, DoubleSupplier xPidInput,
        TrcPidController.PidCoefficients yTippingPidCoeffs, double yTolerance, DoubleSupplier yPidInput)
    {
        if (yTippingPidCoeffs == null || yPidInput == null)
        {
            throw new IllegalArgumentException("yTippingPidParams and yPidInput must not be null.");
        }

        if (xTippingPidCoeffs != null)
        {
            if (xPidInput == null)
            {
                throw new IllegalArgumentException("xPidInput must not be null.");
            }

            xTippingPidCtrl = new TrcPidController("xAntiTippingPidCtrl", xTippingPidCoeffs, xPidInput);
            xTippingPidCtrl.setAbsoluteSetPoint(true);
            xTippingPidCtrl.setTarget(xPidInput.getAsDouble());
            xTippingTolerance = xTolerance;
        }

        yTippingPidCtrl = new TrcPidController("yAntiTippingPidCtrl", yTippingPidCoeffs, yPidInput);
        yTippingPidCtrl.setAbsoluteSetPoint(true);
        yTippingPidCtrl.setTarget(yPidInput.getAsDouble());
        yTippingTolerance = yTolerance;

        antiTippingEnabled = true;
    }   //enableAntiTipping

    /**
     * This method enables anti-tipping drive.
     *
     * @param yTippingPidCoeffs specifies the anti-tipping PID controller coefficients for the Y direction.
     * @param yTolerance specifies the Y tipping tolerance.
     * @param yPidInput specifies the Y PidInput provider.
     */
    public void enableAntiTipping(
        TrcPidController.PidCoefficients yTippingPidCoeffs, double yTolerance, DoubleSupplier yPidInput)
    {
        enableAntiTipping(null, 0.0, null, yTippingPidCoeffs, yTolerance, yPidInput);
    }   //enableAntiTipping

    /**
     * This method disables anti-tipping drive.
     */
    public void disableAntiTipping()
    {
        xTippingPidCtrl = null;
        yTippingPidCtrl = null;
        antiTippingEnabled = false;
    }   //disableAntiTipping

    /**
     * This method checks if anti-tipping drive is enabled.
     *
     * @return true if anti-tipping drive is enabled, false otherwise.
     */
    public boolean isAntiTippingEnabled()
    {
        return antiTippingEnabled;
    }   //isAntiTippingEnabled

    /**
     * This method calculates and returns the anti-tipping power.
     *
     * @param xTippingControl specifies true for the X anti-tipping power, false for the Y anti-tipping power.
     * @return calculated anti-tipping power.
     */
    public double getAntiTippingPower(boolean xTippingControl)
    {
        double power;
        TrcPidController tippingPidCtrl = xTippingControl? xTippingPidCtrl: yTippingPidCtrl;
        double tolerance = xTippingControl? xTippingTolerance: yTippingTolerance;

        power = tippingPidCtrl.calculate();
        if (Math.abs(tippingPidCtrl.getError()) <= tolerance)
        {
            power = 0.0;
        }

        return power;
    }   //getAntiTippingPower

    /**
     * This method checks if it supports holonomic drive. Subclasses that support holonomic drive should override
     * this method.
     *
     * @return true if this drive base supports holonomic drive, false otherwise.
     */
    public boolean supportsHolonomicDrive()
    {
        return false;
    }   //supportsHolonomicDrive

    /**
     * This method returns the number of motors in the drive train.
     *
     * @return number of motors.
     */
    public int getNumMotors()
    {
        return motors.length;
    }   //getNumMotors

    /**
     * This method enables velocity control on the drive motors. When enabled, instead of sending percentage power to
     * the motors, we will send percentage of max motor velocity to the motors.
     *
     * @param maxMotorVel specifies the maximum velocity of the motor in scaled unit/sec if enabled.
     *        Generally, drive motors are not scaled, so the unit is really motor native unit. In FRC, this would be
     *        rotation/sec. In FTC, this would be encoder counts/sec.
     * @param velPidCoeffs specifies the velocity control PID coefficients.
     * @param velPidTolerance specifies the PID tolerance.
     * @param softwarePid specifies true to use software PID, false to use motor native PID.
     */
    public void enableMotorVelocityControl(
        double maxMotorVel, TrcPidController.PidCoefficients velPidCoeffs, double velPidTolerance, boolean softwarePid)
    {
        for (TrcMotor motor : motors)
        {
            motor.enablePercentageVelocityControl(maxMotorVel, velPidCoeffs, velPidTolerance, softwarePid);
        }
    }   //enableMotorVelocityControl

    /**
     * This method disables velocity control on the drive motors. When disabled, we will send percentage power to the
     * motors.
     */
    public void disableMotorVelocityControl()
    {
        for (TrcMotor motor : motors)
        {
            motor.disablePercentageVelocityControl();
        }
    }   //disableMotorVelocityControl

    /**
     * This method inverts direction of a given motor in the drive train.
     *
     * @param index    specifies the index in the motors array.
     * @param inverted specifies true if inverting motor direction.
     */
    protected void setInvertedMotor(int index, boolean inverted)
    {
        motors[index].setMotorInverted(inverted);
    }   //setInvertedMotor

    /**
     * This method is called by the subclass to set the stall detection velocity threshold value.
     *
     * @param stallVelThreshold specifies the stall detection velocity threshold value.
     */
    protected void setStallVelocityThreshold(double stallVelThreshold)
    {
        this.stallVelThreshold = Math.abs(stallVelThreshold);
    }   //setStallVelocityThreshold

    /**
     * This method determines if the drive base is stalled. A drive base is in stalled state when there is power
     * applied to the wheels and odometry has no movement for the given period of time.
     *
     * @param stallTime specifies the stall time in seconds.
     * @return true if the drive base is in stalled state, false otherwise.
     */
    public boolean isStalled(double stallTime)
    {
        // stallStartTime is set to current time whenever there is wheel movement.
        // stallStartTime is reset to zero whenever the drive base is stopped (i.e. power set to zero).
        return stallStartTime != 0.0 && TrcTimer.getCurrentTime() > stallStartTime + stallTime;
    }   //isStalled

    /**
     * This method enables/disables brake mode of the drive base.
     *
     * @param enabled specifies true to enable brake mode, false to disable it.
     */
    public void setBrakeMode(boolean enabled)
    {
        for (TrcMotor motor : motors)
        {
            motor.setBrakeModeEnabled(enabled);
        }
    }   //setBrakeMode

    /**
     * This methods stops the drive base.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *              ownership aware.
     */
    public void stop(String owner)
    {
        tracer.traceDebug(moduleName, "owner=" + owner);
        if (validateOwnership(owner))
        {
            if (driveOwner != null)
            {
                driveTimer.cancel();
                if (driveTimerEvent != null)
                {
                    driveTimerEvent.cancel();
                    driveTimerEvent = null;
                }
                driveOwner = null;
            }

            for (TrcMotor motor : motors)
            {
                motor.setPower(0.0);
            }
        }
    }   //stop

    /**
     * This methods stops the drive base.
     */
    public void stop()
    {
        stop(null);
    }   //stop

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param owner      specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                   ownership aware.
     * @param leftPower  specifies left power value.
     * @param rightPower specifies right power value.
     * @param driveTime  specifies the amount of time in seconds after which the drive base will stop.
     * @param event      specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void tankDrive(String owner, double leftPower, double rightPower, double driveTime, TrcEvent event)
    {
        tankDrive(owner, leftPower, rightPower, false, driveTime, event);
    }   //tankDrive

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param owner      specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                   ownership aware.
     * @param leftPower  specifies left power value.
     * @param rightPower specifies right power value.
     */
    public void tankDrive(String owner, double leftPower, double rightPower)
    {
        tankDrive(owner, leftPower, rightPower, false, 0.0, null);
    }   //tankDrive

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower  specifies left power value.
     * @param rightPower specifies right power value.
     * @param driveTime  specifies the amount of time in seconds after which the drive base will stop.
     * @param event      specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void tankDrive(double leftPower, double rightPower, double driveTime, TrcEvent event)
    {
        tankDrive(null, leftPower, rightPower, false, driveTime, event);
    }   //tankDrive

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower  specifies left power value.
     * @param rightPower specifies right power value.
     */
    public void tankDrive(double leftPower, double rightPower)
    {
        tankDrive(null, leftPower, rightPower, false, 0.0, null);
    }   //tankDrive

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower  specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted   specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void tankDrive(double leftPower, double rightPower, boolean inverted)
    {
        tankDrive(null, leftPower, rightPower, inverted, 0.0, null);
    }   //tankDrive

    /**
     * This method drives the motors at "magnitude" and "curve". Both magnitude and curve are -1.0 to +1.0 values,
     * where 0.0 represents stopped and not turning. curve less than 0 will turn left and curve greater than 0 will
     * turn right. The algorithm for steering provides a constant turn radius for any normal speed range, both
     * forward and backward. Increasing sensitivity causes sharper turns for fixed values of curve.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param magnitude specifies the speed setting for the outside wheel in a turn, forward or backwards, +1 to -1.
     * @param curve     specifies the rate of turn, constant for different forward speeds. Set curve less than 0 for left
     *                  turn or curve greater than 0 for right turn. Set curve = e^(-r/w) to get a turn radius r for
     *                  wheel base w of your robot. Conversely, turn radius r = -ln(curve)*w for a given value of curve
     *                  and wheel base w.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void curveDrive(
        String owner, double magnitude, double curve, boolean inverted, double driveTime, TrcEvent event)
    {
        double leftOutput;
        double rightOutput;

        tracer.traceDebug(
            moduleName,
            "owner=" + owner +
            ",mag=" + magnitude +
            ",curve=" + curve +
            ",inverted=" + inverted +
            ",driveTime=" + driveTime +
            ",event=" + event);
        if (validateOwnership(owner))
        {
            if (curve < 0.0)
            {
                double value = Math.log(-curve);
                double ratio = (value - sensitivity) / (value + sensitivity);
                if (ratio == 0.0)
                {
                    ratio = 0.0000000001;
                }
                leftOutput = magnitude / ratio;
                rightOutput = magnitude;
            }
            else if (curve > 0.0)
            {
                double value = Math.log(curve);
                double ratio = (value - sensitivity) / (value + sensitivity);
                if (ratio == 0.0)
                {
                    ratio = 0.0000000001;
                }
                leftOutput = magnitude;
                rightOutput = magnitude / ratio;
            }
            else
            {
                leftOutput = magnitude;
                rightOutput = magnitude;
            }

            tankDrive(owner, leftOutput, rightOutput, inverted, driveTime, event);
        }
    }   //curveDrive

    /**
     * This method drives the motors at "magnitude" and "curve". Both magnitude and curve are -1.0 to +1.0 values,
     * where 0.0 represents stopped and not turning. curve less than 0 will turn left and curve greater than 0 will
     * turn right. The algorithm for steering provides a constant turn radius for any normal speed range, both
     * forward and backward. Increasing sensitivity causes sharper turns for fixed values of curve.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param magnitude specifies the speed setting for the outside wheel in a turn, forward or backwards, +1 to -1.
     * @param curve     specifies the rate of turn, constant for different forward speeds. Set curve less than 0 for left
     *                  turn or curve greater than 0 for right turn. Set curve = e^(-r/w) to get a turn radius r for
     *                  wheel base w of your robot. Conversely, turn radius r = -ln(curve)*w for a given value of curve
     *                  and wheel base w.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void curveDrive(String owner, double magnitude, double curve, boolean inverted)
    {
        curveDrive(owner, magnitude, curve, inverted, 0.0, null);
    }   //curveDrive

    /**
     * This method drives the motors at "magnitude" and "curve". Both magnitude and curve are -1.0 to +1.0 values,
     * where 0.0 represents stopped and not turning. curve less than 0 will turn left and curve greater than 0 will
     * turn right. The algorithm for steering provides a constant turn radius for any normal speed range, both
     * forward and backward. Increasing sensitivity causes sharper turns for fixed values of curve.
     *
     * @param magnitude specifies the speed setting for the outside wheel in a turn, forward or backwards, +1 to -1.
     * @param curve     specifies the rate of turn, constant for different forward speeds. Set curve less than 0 for left
     *                  turn or curve greater than 0 for right turn. Set curve = e^(-r/w) to get a turn radius r for
     *                  wheel base w of your robot. Conversely, turn radius r = -ln(curve)*w for a given value of curve
     *                  and wheel base w.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void curveDrive(double magnitude, double curve, boolean inverted, double driveTime, TrcEvent event)
    {
        curveDrive(null, magnitude, curve, inverted, driveTime, event);
    }   //curveDrive

    /**
     * This method drives the motors with the given magnitude and curve values.
     *
     * @param magnitude specifies the magnitude value.
     * @param curve     specifies the curve value.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void curveDrive(double magnitude, double curve, double driveTime, TrcEvent event)
    {
        curveDrive(null, magnitude, curve, false, driveTime, event);
    }   //curveDrive

    /**
     * This method drives the motors with the given magnitude and curve values.
     *
     * @param magnitude specifies the magnitude value.
     * @param curve     specifies the curve value.
     */
    public void curveDrive(double magnitude, double curve)
    {
        curveDrive(null, magnitude, curve, false, 0.0, null);
    }   //curveDrive

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param owner      specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                   ownership aware.
     * @param drivePower specifies the drive power value.
     * @param turnPower  specifies the turn power value.
     * @param inverted   specifies true to invert control (i.e. robot front becomes robot back).
     * @param driveTime  specifies the amount of time in seconds after which the drive base will stop.
     * @param event      specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void arcadeDrive(
        String owner, double drivePower, double turnPower, boolean inverted, double driveTime, TrcEvent event)
    {
        double leftPower;
        double rightPower;

        tracer.traceDebug(
            moduleName,
            "owner=" + owner +
            ",drivePower=" + drivePower +
            ",turnPower=" + turnPower +
            ",inverted=" + inverted +
            ",driveTime=" + driveTime +
            ",event=" + event);
        if (validateOwnership(owner))
        {
            drivePower = TrcUtil.clipRange(drivePower);
            turnPower = TrcUtil.clipRange(turnPower);

            leftPower = drivePower + turnPower;
            rightPower = drivePower - turnPower;
            double maxMag = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxMag > 1.0)
            {
                leftPower /= maxMag;
                rightPower /= maxMag;
            }

            tankDrive(owner, leftPower, rightPower, inverted, driveTime, event);
        }
    }   //arcadeDrive

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param owner      specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                   ownership aware.
     * @param drivePower specifies the drive power value.
     * @param turnPower  specifies the turn power value.
     * @param inverted   specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void arcadeDrive(String owner, double drivePower, double turnPower, boolean inverted)
    {
        arcadeDrive(owner, drivePower, turnPower, inverted, 0.0, null);
    }   //arcadeDrive

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower  specifies the turn power value.
     * @param inverted   specifies true to invert control (i.e. robot front becomes robot back).
     * @param driveTime  specifies the amount of time in seconds after which the drive base will stop.
     * @param event      specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void arcadeDrive(double drivePower, double turnPower, boolean inverted, double driveTime, TrcEvent event)
    {
        arcadeDrive(null, drivePower, turnPower, inverted, driveTime, event);
    }   //arcadeDrive

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower  specifies the turn power value.
     * @param driveTime  specifies the amount of time in seconds after which the drive base will stop.
     * @param event      specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void arcadeDrive(double drivePower, double turnPower, double driveTime, TrcEvent event)
    {
        arcadeDrive(null, drivePower, turnPower, false, driveTime, event);
    }   //arcadeDrive

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower  specifies the turn power value.
     */
    public void arcadeDrive(double drivePower, double turnPower)
    {
        arcadeDrive(null, drivePower, turnPower, false, 0.0, null);
    }   //arcadeDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain. Subclasses that supports holonomic drive should
     * override this method.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive(
        String owner, double x, double y, double rotation, boolean inverted, double gyroAngle, double driveTime,
        TrcEvent event)
    {
        throw new UnsupportedOperationException("Holonomic drive is not supported by this drive base!");
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain. Subclasses that supports holonomic drive should
     * override this method.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     */
    public void holonomicDrive(String owner, double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        holonomicDrive(owner, x, y, rotation, inverted, gyroAngle, 0.0, null);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive(
        String owner, double x, double y, double rotation, boolean inverted, double driveTime, TrcEvent event)
    {
        holonomicDrive(owner, x, y, rotation, inverted, 0.0, driveTime, event);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner    specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                 ownership aware.
     * @param x        specifies the x power.
     * @param y        specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void holonomicDrive(String owner, double x, double y, double rotation, boolean inverted)
    {
        holonomicDrive(owner, x, y, rotation, inverted, 0.0, 0.0, null);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner    specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                 ownership aware.
     * @param x        specifies the x power.
     * @param y        specifies the y power.
     * @param rotation specifies the rotating power.
     */
    public void holonomicDrive(String owner, double x, double y, double rotation)
    {
        holonomicDrive(owner, x, y, rotation, false, 0.0, 0.0, null);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive(
        double x, double y, double rotation, boolean inverted, double driveTime, TrcEvent event)
    {
        holonomicDrive(null, x, y, rotation, inverted, 0.0, driveTime, event);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x        specifies the x power.
     * @param y        specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void holonomicDrive(double x, double y, double rotation, boolean inverted)
    {
        holonomicDrive(null, x, y, rotation, inverted, 0.0, 0.0, null);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive(
        String owner, double x, double y, double rotation, double gyroAngle, double driveTime, TrcEvent event)
    {
        holonomicDrive(owner, x, y, rotation, false, gyroAngle, driveTime, event);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     */
    public void holonomicDrive(String owner, double x, double y, double rotation, double gyroAngle)
    {
        holonomicDrive(owner, x, y, rotation, false, gyroAngle, 0.0, null);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive(
        double x, double y, double rotation, double gyroAngle, double driveTime, TrcEvent event)
    {
        holonomicDrive(null, x, y, rotation, false, gyroAngle, driveTime, event);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive(String owner, double x, double y, double rotation, double driveTime, TrcEvent event)
    {
        holonomicDrive(owner, x, y, rotation, false, 0.0, driveTime, event);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive(double x, double y, double rotation, double driveTime, TrcEvent event)
    {
        holonomicDrive(null, x, y, rotation, false, 0.0, driveTime, event);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x        specifies the x power.
     * @param y        specifies the y power.
     * @param rotation specifies the rotating power.
     */
    public void holonomicDrive(double x, double y, double rotation)
    {
        holonomicDrive(null, x, y, rotation, false, 0.0, 0.0, null);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees. 0 is forward. Positive is clockwise.
     * @param rotation  specifies the rotation power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive_Polar(String owner, double magnitude, double direction, double rotation,
        boolean inverted, double driveTime, TrcEvent event)
    {
        double dirInRads = Math.toRadians(direction);
        holonomicDrive(
            owner, magnitude * Math.sin(dirInRads), magnitude * Math.cos(dirInRads), rotation, inverted, 0.0,
            driveTime, event);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation  specifies the rotation power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive_Polar(
        double magnitude, double direction, double rotation, boolean inverted, double driveTime, TrcEvent event)
    {
        holonomicDrive_Polar(null, magnitude, direction, rotation, inverted, driveTime, event);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation  specifies the rotation power.
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive_Polar(
        String owner, double magnitude, double direction, double rotation, double gyroAngle, double driveTime,
        TrcEvent event)
    {
        double dirInRads = Math.toRadians(direction);
        holonomicDrive(
            owner, magnitude * Math.sin(dirInRads), magnitude * Math.cos(dirInRads), rotation, false, gyroAngle,
            driveTime, event);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees. 0 is forward. Positive is clockwise.
     * @param rotation  specifies the rotation power.
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive_Polar(
        double magnitude, double direction, double rotation, double gyroAngle, double driveTime, TrcEvent event)
    {
        holonomicDrive_Polar(null, magnitude, direction, rotation, gyroAngle, driveTime, event);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees. 0 is forward. Positive is clockwise.
     * @param rotation  specifies the rotation power.
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     */
    public void holonomicDrive_Polar(double magnitude, double direction, double rotation, double gyroAngle)
    {
        holonomicDrive_Polar(null, magnitude, direction, rotation, gyroAngle, 0.0, null);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation  specifies the rotation power.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive_Polar(
        String owner, double magnitude, double direction, double rotation, double driveTime, TrcEvent event)
    {
        double dirInRads = Math.toRadians(direction);
        holonomicDrive(
            owner, magnitude * Math.sin(dirInRads), magnitude * Math.cos(dirInRads), rotation, false, 0.0, driveTime,
            event);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees. 0 is forward. Positive is clockwise.
     * @param rotation  specifies the rotation power.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    public void holonomicDrive_Polar(
        double magnitude, double direction, double rotation, double driveTime, TrcEvent event)
    {
        holonomicDrive_Polar(null, magnitude, direction, rotation, driveTime, event);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees. 0 is forward. Positive is clockwise.
     * @param rotation  specifies the rotation power.
     */
    public void holonomicDrive_Polar(double magnitude, double direction, double rotation)
    {
        holonomicDrive_Polar(null, magnitude, direction, rotation, 0.0, null);
    }   //holonomicDrive_Polar

    /**
     * This method is called periodically to update the drive base odometry (position and velocity of both x, y and
     * angle).
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void odometryTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        synchronized (odometry)
        {
            Odometry odometryDelta;

            if (odometryWheels != null)
            {
                odometryDelta = odometryWheels.getOdometryDelta();
                updateOdometry(odometryDelta, odometry.position.angle);
            }
            else if (absoluteOdometry != null)
            {
                absoluteOdometry.updateCache();
                TrcPose2D currPos = absoluteOdometry.getPosition();
                odometry.position.x = currPos.x;
                odometry.position.y = currPos.y;
                odometry.position.angle =
                    absOdoHeadingWrapConverter != null? absOdoHeadingWrapConverter.getContinuousValue(): currPos.angle;
                TrcPose2D currVel = absoluteOdometry.getVelocity();
                odometry.velocity.x = currVel.x;
                odometry.velocity.y = currVel.y;
                odometry.velocity.angle = currVel.angle;
            }
            else
            {
                //
                // Update all motor states.
                //
                for (int i = 0; i < motors.length; i++)
                {
                    motorsState.prevMotorOdometries[i] = motorsState.currMotorOdometries[i];
                    motorsState.currMotorOdometries[i] = motors[i].getOdometry();
                }

                if (synchronizeOdometries)
                {
                    synchronizeOdometries(motorsState.currMotorOdometries);
                }
                //
                // Calculate pose delta from last pose and update odometry accordingly.
                //
                odometryDelta = getOdometryDelta(motorsState.prevMotorOdometries, motorsState.currMotorOdometries);
                if (gyro != null)
                {
                    TrcOdometrySensor.Odometry gyroOdometry = gyro.getOdometry();

                    if (SYNC_GYRO_DATA)
                    {
                        tracer.traceDebug(
                            moduleName,
                            "Gyro Before: timestamp=" + gyroOdometry.currTimestamp +
                            ", pos=" + gyroOdometry.currPos +
                            ", vel=" + gyroOdometry.velocity);
                        double refTimestamp = motorsState.currMotorOdometries[0].currTimestamp;
                        gyroOdometry.currPos -= gyroOdometry.velocity * (gyroOdometry.currTimestamp - refTimestamp);
                        gyroOdometry.currTimestamp = refTimestamp;
                        tracer.traceDebug(
                            moduleName,
                            "Gyro After: timestamp=" + gyroOdometry.currTimestamp +
                            ", pos=" + gyroOdometry.currPos +
                            ", vel=" + gyroOdometry.velocity);
                    }
                    // Overwrite the angle/turnrate values if gyro present, since that's more accurate
                    odometryDelta.position.angle = gyroOdometry.currPos - gyroOdometry.prevPos;
                    odometryDelta.velocity.angle = gyroOdometry.velocity;
                }

                updateOdometry(odometryDelta, odometry.position.angle);
                tracer.traceDebug(
                    moduleName,
                    "motorsState=" + motorsState +
                    ", delta=" + odometryDelta +
                    ", odometry=" + odometry);
            }

            if (TrcUtil.magnitude(odometry.velocity.x, odometry.velocity.y) > stallVelThreshold)
            {
                // reset stall start time to current time if drive base has movement.
                stallStartTime = TrcTimer.getCurrentTime();
            }
        }
    }   //odometryTask

    /**
     * This method is called when the competition mode is about to end to stop the drive base.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void stopTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        stop();
    }   //stopTask

    /**
     * This method synchronizes the odometries of all the drive base motors. Since all motors are read at different
     * times, the delay may cause inconsistencies on the odometry data which will affect the accuracy of the drive
     * base odometry calculation. We will make all motors based off of the timestamp of the last motor and their
     * position data will be interpolated accordingly. In effect, we are fast forwarding the other motors to sync
     * with the latest timestamp and predicting their positions with their velocity info.
     *
     * @param odometries specifies the array of odometries of all drive base motors.
     */
    private void synchronizeOdometries(TrcOdometrySensor.Odometry[] odometries)
    {
        int lastIndex = odometries.length - 1;
        double refTimestamp = odometries[lastIndex].currTimestamp;

        for (int i = 0; i < lastIndex; i++)
        {
            tracer.traceDebug(
                moduleName,
                "[" + i +
                "] Before: name=" + odometries[i].sensor +
                ", timestamp=" + odometries[i].currTimestamp +
                ", pos=" + odometries[i].currPos +
                ", vel=" + odometries[i].velocity);
            odometries[i].currPos -= odometries[i].velocity * (odometries[i].currTimestamp - refTimestamp);
            odometries[i].currTimestamp = refTimestamp;
            tracer.traceDebug(
                moduleName,
                "[" + i +
                "] After: name=" + odometries[i].sensor +
                ", timestamp=" + odometries[i].currTimestamp +
                ", pos=" + odometries[i].currPos +
                ", vel=" + odometries[i].velocity);
        }
        tracer.traceDebug(
            moduleName,
            "[" + lastIndex +
            "] Reference: name=" + odometries[lastIndex].sensor +
            ", timestamp=" + odometries[lastIndex].currTimestamp +
            ", pos=" + odometries[lastIndex].currPos +
            ", vel=" + odometries[lastIndex].velocity);
    }   //synchronizeOdometries

    /**
     * This method updates the current robot odometry with the delta either using 0 or 1st order dynamics depending on
     * the value of <code>USE_CURVED_PATH</code>. If true, use a curved path (with nonzero curvature) otherwise model
     * path as a bunch of straight lines. The curved path is more accurate.
     *
     * @param delta specifies the odometry delta since the last update.
     * @param angle specifies the robot angle in the last update.
     */
    private void updateOdometry(Odometry delta, double angle)
    {
        if (USE_CURVED_PATH)
        {
            // The math below uses a different coordinate system (NWU) so we have to convert
            double[] posArr = enuToNwuChangeOfBasis.operate(new double[] { delta.position.x, delta.position.y });
            double x = posArr[0];
            double y = posArr[1];
            // Convert clockwise degrees to counter-clockwise radians
            double theta = Math.toRadians(-delta.position.angle);
            double headingRad = Math.toRadians(-angle);

            // The derivation of the following math is here in section 11.1
            // (https://file.tavsys.net/control/state-space-guide.pdf)
            // A is a transformation matrix representing a CCW rotation by headingRad radians
            // This is used to bring the change in pose into the global reference frame
            RealMatrix A = MatrixUtils.createRealMatrix(
                new double[][] { { Math.cos(headingRad), -Math.sin(headingRad), 0 },
                                 { Math.sin(headingRad), Math.cos(headingRad), 0 },
                                 { 0, 0, 1 } });
            // B is used to apply a nonzero curvature to the path. When the curvature is zero, B resolves to the
            // identity matrix.
            // The math involved isn't immediately intuitive, but it's basically the integration of the forward odometry
            // matrix equation.
            RealMatrix B;
            if (Math.abs(theta) <= 1E-9)
            {
                // Use the taylor series approximations, since some values are indeterminate
                B = MatrixUtils.createRealMatrix(new double[][] { { 1 - theta * theta / 6.0, -theta / 2.0, 0 },
                    { theta / 2.0, 1 - theta * theta / 6.0, 0 }, { 0, 0, 1 } });
            }
            else
            {
                B = MatrixUtils.createRealMatrix(new double[][] { { Math.sin(theta), Math.cos(theta) - 1, 0 },
                    { 1 - Math.cos(theta), Math.sin(theta), 0 }, { 0, 0, theta } });
                B = B.scalarMultiply(1.0 / theta);
            }
            // C is the column vector containing the "raw" change in pose. This is the immediate output of the forward
            // odometry multiplied by timestep
            RealVector C = MatrixUtils.createRealVector(new double[] { x, y, theta });
            // Get the change in global pose
            RealVector globalPose = A.multiply(B).operate(C);
            // Convert back to our (ENU) reference frame
            RealVector pos = nwuToEnuChangeOfBasis.operate(globalPose.getSubVector(0, 2));
            // Convert back to clockwise degrees for angle
            theta = Math.toDegrees(-globalPose.getEntry(2));

            // Rotate the velocity vector into the global reference frame
            RealVector vel = MatrixUtils.createRealVector(new double[] { delta.velocity.x, delta.velocity.y });
            vel = TrcUtil.rotateCW(vel, angle);

            // Update the odometry values
            odometry.position.x += pos.getEntry(0);
            odometry.position.y += pos.getEntry(1);
            odometry.position.angle += theta;
            odometry.velocity.x = vel.getEntry(0);
            odometry.velocity.y = vel.getEntry(1);
            odometry.velocity.angle = delta.velocity.angle;
        }
        else
        {
            RealVector pos = MatrixUtils.createRealVector(new double[] { delta.position.x, delta.position.y });
            RealVector vel = MatrixUtils.createRealVector(new double[] { delta.velocity.x, delta.velocity.y });

            pos = TrcUtil.rotateCW(pos, odometry.position.angle);
            vel = TrcUtil.rotateCW(vel, odometry.position.angle);

            odometry.position.x += pos.getEntry(0);
            odometry.position.y += pos.getEntry(1);
            odometry.velocity.x = vel.getEntry(0);
            odometry.velocity.y = vel.getEntry(1);
            odometry.position.angle += delta.position.angle;
            odometry.velocity.angle = delta.velocity.angle;
        }
    }   //updateOdometry

}   //class TrcDriveBase
