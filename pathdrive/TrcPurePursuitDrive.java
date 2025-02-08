/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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

import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.dataprocessor.TrcWarpSpace;
import trclib.drivebase.TrcDriveBase;
import trclib.driverio.TrcTone;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.sensor.TrcRobotBattery;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent Pure Pursuit drive for holonomic or non-holonomic robots.
 * Essentially, a pure pursuit drive navigates the robot to chase a point along the path. The point to chase is
 * chosen by intersecting a lookahead circle centered on the robot with a specific radius with the path, and chasing
 * the "furthest" intersection. The smaller the radius is, the more "tightly" the robot will follow a path, but it
 * will be more prone to oscillation and sharp turns. A larger radius will tend to smooth out turns and corners. Note
 * that the error tolerance must be less than the lookahead radius, so choose them accordingly.
 * <p>
 * A path consists of an array of waypoints, specifying position, velocity, and heading. All other properties
 * of the TrcWaypoint object may be ignored.The path may be low resolution, as this automatically interpolates between
 * waypoints. If you want the robot to maintain heading to a fixed target, call enableFixedHeading with the target
 * heading offset and it will ignore all the heading values. Otherwise, call disableFixedHeading, ensure that the
 * heading tolerance and pid coefficients are set, and it will follow the heading values specified by the path. Note
 * that FixedHeading is only supported for holonomic robots.
 * <p>
 * A somewhat similar idea is here:
 * <a href="https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552">...</a>
 * or <a href="https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf">...</a>
 * <p>
 * Note that this paper is for non-holonomic robots. This means that all the turning radius stuff isn't very relevant.
 * Technically, we could impose limits on the turning radius as a function of robot velocity and max rot vel, but that's
 * unnecessarily complicated, in my view. Additionally, it does point injection instead of interpolation, and path
 * smoothing, which we don't do, since a nonzero lookahead radius will naturally smooth it anyway.
 */
public class TrcPurePursuitDrive
{
    private static final boolean INVERTED_TARGET = false;
    private static final boolean applySquidOnVel = false;
    private static final boolean fastMode = true;
    private static final boolean constantVelInLookahead = true;
    private static final boolean useLookaheadVelUntilDecel = false;     //algorithm 2

    public interface WaypointEventHandler
    {
        /**
         * This method is called when Pure Pursuit crosses a waypoint or the path is completed.
         *
         * @param index specifies the index of the waypoint in the path, -1 if the path is completed or canceled.
         * @param waypoint specifies the current target waypoint.
         */
        void waypointEvent(int index, TrcWaypoint waypoint);
    }   //interface WaypointEventHandler

    public interface TargetHeadingOffset
    {
        /**
         * This method is called when fixed heading mode is enabled to get the fixed heading offset from the current
         * robot heading.
         *
         * @return target heading offset from current robot heading.
         */
        Double getOffset();

    }   //interface TargetHeadingOffset

    public enum InterpolationType
    {
        LINEAR(1), QUADRATIC(2), CUBIC(3), QUARTIC(4), QUADRATIC_INV(2), CUBIC_INV(3), QUARTIC_INV(4);

        final int value;

        InterpolationType(int value)
        {
            this.value = value;
        }   //InterpolationType

    }   //enum InterpolationType

    public TrcDbgTrace tracer;
    private String instanceName;
    private TrcDriveBase driveBase;
    private volatile double lookaheadRadius;    // Volatile so it can be changed at runtime
    private volatile double posTolerance;       // Volatile so it can be changed at runtime
    private volatile double turnTolerance;      // Volatile so it can be changed at runtime
    private TrcPidController xPosPidCtrl, yPosPidCtrl, turnPidCtrl, velPidCtrl;
    private TrcWarpSpace warpSpace;
    private TrcTaskMgr.TaskObject driveTaskObj;
    // Tracer config.
    private boolean logRobotPoseEvents = false;
    private boolean tracePidInfo = false;
    private boolean verbosePidInfo = false;
    private TrcRobotBattery battery = null;

    private static final double DEF_BEEP_FREQUENCY = 880.0; //in Hz
    private static final double DEF_BEEP_DURATION = 0.2;    //in seconds
    private TrcTone beepDevice = null;
    private double beepFrequency = DEF_BEEP_FREQUENCY;
    private double beepDuration = DEF_BEEP_DURATION;

    private double moveOutputLimit = Double.POSITIVE_INFINITY;
    private double rotOutputLimit = Double.POSITIVE_INFINITY;
    private WaypointEventHandler waypointEventHandler = null;
    private InterpolationType interpolationType = InterpolationType.LINEAR;
    private volatile boolean incrementalTurn;
    private TargetHeadingOffset targetHeadingOffset = null;
    private volatile boolean stalled = false;

    private String owner = null;
    private TrcPath path;
    private TrcEvent onFinishedEvent;
    private double timedOutTime;
    private int pathIndex;
    private TrcPose2D referencePose;
    private TrcPose2D targetPose;
    private Double targetVelocity = null;
    private boolean fastModeEnabled = false;
    private boolean resetError = false;
    private Double firstLookaheadVel = null;
    private boolean nearEndTarget = false;
    private int prevIndex = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the reference to the drive base.
     * @param lookaheadRadius specifies the distance between the robot and next following point.
     * @param posTolerance specifies the position tolerance.
     * @param turnTolerance specifies the turn tolerance.
     * @param xPidCtrl specifies the position PID controller for X.
     * @param yPidCtrl specifies the position PID controller for Y.
     * @param turnPidCtrl specifies the turn PID controller.
     * @param velPidCtrl specifies the velocity PID controller.
     */
    public void commonInit(
        String instanceName, TrcDriveBase driveBase, double lookaheadRadius, double posTolerance, double turnTolerance,
        TrcPidController xPidCtrl, TrcPidController yPidCtrl, TrcPidController turnPidCtrl,
        TrcPidController velPidCtrl)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.driveBase = driveBase;
        this.xPosPidCtrl = xPidCtrl;
        this.yPosPidCtrl = yPidCtrl;
        this.turnPidCtrl = turnPidCtrl;
        this.velPidCtrl = velPidCtrl;
        incrementalTurn = xPidCtrl != null;
        // If INVERTED_TARGET is true, use Abhay's way to set target to zero and just keep changing getInput.
        if (INVERTED_TARGET)
        {
            if (xPosPidCtrl != null)
            {
                xPosPidCtrl.setAbsoluteSetPoint(true);
                xPosPidCtrl.setInverted(true);
            }
            yPosPidCtrl.setAbsoluteSetPoint(true);
            yPosPidCtrl.setInverted(true);
        }
        turnPidCtrl.setAbsoluteSetPoint(true);
        turnPidCtrl.setNoOscillation(false);
        // We are not checking velocity being onTarget, so we don't need velocity tolerance.
        velPidCtrl.setAbsoluteSetPoint(true);

        setPositionToleranceAndLookaheadRadius(posTolerance, lookaheadRadius);
        this.turnTolerance = turnTolerance;

        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);
        driveTaskObj = TrcTaskMgr.createTask(instanceName + ".driveTask", this::driveTask);
    }   //commonInit

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the reference to the drive base.
     * @param lookaheadRadius specifies the distance between the robot and next following point.
     * @param posTolerance specifies the position tolerance.
     * @param turnTolerance specifies the turn tolerance.
     * @param xPidCtrl specifies the position PID controller for X.
     * @param yPidCtrl specifies the position PID controller for Y.
     * @param turnPidCtrl specifies the turn PID controller.
     * @param velPidCtrl specifies the velocity PID controller.
     */
    public TrcPurePursuitDrive(
        String instanceName, TrcDriveBase driveBase, double lookaheadRadius, double posTolerance, double turnTolerance,
        TrcPidController xPidCtrl, TrcPidController yPidCtrl, TrcPidController turnPidCtrl,
        TrcPidController velPidCtrl)
    {
        if (xPidCtrl != null && !driveBase.supportsHolonomicDrive())
        {
            throw new IllegalArgumentException(
                "xPosPidCtrl is provided but drive base does not support holonomic drive!");
        }

        commonInit(
            instanceName, driveBase, lookaheadRadius, posTolerance, turnTolerance, xPidCtrl, yPidCtrl, turnPidCtrl,
            velPidCtrl);
    }   //TrcPurePursuitDrive

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the reference to the drive base.
     * @param lookaheadRadius specifies the distance between the robot and next following point.
     * @param posTolerance specifies the position tolerance.
     * @param turnTolerance specifies the turn tolerance.
     * @param xPosPidCoeff specifies the position PID coefficients for X.
     * @param yPosPidCoeff specifies the position PID coefficients for Y.
     * @param turnPidCoeff specifies the turn PID coefficients.
     * @param velPidCoeff specifies the velocity PID coefficients.
     */
    public TrcPurePursuitDrive(
        String instanceName, TrcDriveBase driveBase, double lookaheadRadius, double posTolerance, double turnTolerance,
        TrcPidController.PidCoefficients xPosPidCoeff, TrcPidController.PidCoefficients yPosPidCoeff,
        TrcPidController.PidCoefficients turnPidCoeff, TrcPidController.PidCoefficients velPidCoeff)
//        TrcPidController.FFCoefficients velFfCoeff)
    {
        if (xPosPidCoeff != null && !driveBase.supportsHolonomicDrive())
        {
            throw new IllegalArgumentException(
                "xPosPidCoeff is provided but drive base does not support holonomic drive!");
        }

        TrcPidController xPidCtrl, yPidCtrl, turnPidCtrl, velPidCtrl;
        // If INVERTED_TARGET is true, use Abhay's way to set target to zero and just keep changing getInput.
        if (INVERTED_TARGET)
        {
            xPidCtrl = xPosPidCoeff == null ? null :
                new TrcPidController(instanceName + ".xPosPid", xPosPidCoeff, this::getXPosition);
            yPidCtrl = new TrcPidController(instanceName + ".yPosPid", yPosPidCoeff, this::getYPosition);
        }
        else
        {
            xPidCtrl = xPosPidCoeff == null ? null :
                new TrcPidController(instanceName + ".xPosPid", xPosPidCoeff, driveBase::getXPosition);
            yPidCtrl = new TrcPidController(instanceName + ".yPosPid", yPosPidCoeff, driveBase::getYPosition);
        }
        turnPidCtrl = new TrcPidController(instanceName + ".turnPid", turnPidCoeff, driveBase::getHeading);
        // We are not checking velocity being onTarget, so we don't need velocity tolerance.
        velPidCtrl = new TrcPidController(instanceName + ".velPid", velPidCoeff, this::getRobotVelocity);

        commonInit(
            instanceName, driveBase, lookaheadRadius, posTolerance, turnTolerance, xPidCtrl, yPidCtrl, turnPidCtrl,
            velPidCtrl);
    }   //TrcPurePursuitDrive

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the X position PID controller created for Pure Pursuit Drive.
     *
     * @return X position PID controller.
     */
    public TrcPidController getXPosPidCtrl()
    {
        return xPosPidCtrl;
    }   //getXPosPidCtrl

    /**
     * This method returns the Y position PID controller created for the Pure Pursuit Drive.
     *
     * @return Y position PID controller.
     */
    public TrcPidController getYPosPidCtrl()
    {
        return yPosPidCtrl;
    }   //getYPosPidCtrl

    /**
     * This method returns the turn PID controller created for the Pure Pursuit Drive.
     *
     * @return turn PID controller.
     */
    public TrcPidController getTurnPidCtrl()
    {
        return turnPidCtrl;
    }   //getTurnPidCtrl

    /**
     * This method enables/disables the mode that square rooting all PID output. By square rooting the PID output,
     * it gives a boost to the output when the error is smaller. That means it will make PID stronger to reach
     * target.
     *
     * @param enable specifies true to enable and false to disable.
     */
    public synchronized void setSquareRootPidEnabled(boolean enable)
    {
        if (xPosPidCtrl != null)
        {
            xPosPidCtrl.setSquareRootOutputEnabled(enable);
        }
        yPosPidCtrl.setSquareRootOutputEnabled(enable);
        turnPidCtrl.setSquareRootOutputEnabled(enable);
        if (applySquidOnVel)
        {
            velPidCtrl.setSquareRootOutputEnabled(enable);
        }
    }   //setSquareRootPidEnabled

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param msgLevel specifies the message level.
     * @param logRobotPoseEvents specifies true to log robot pose events, false otherwise.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     * @param verbosePidInfo specifies true to trace verbose PID info, false otherwise.
     * @param battery specifies the battery object to get battery info for the message.
     */
    public synchronized void setTraceLevel(
        TrcDbgTrace.MsgLevel msgLevel, boolean logRobotPoseEvents, boolean tracePidInfo, boolean verbosePidInfo,
        TrcRobotBattery battery)
    {
        tracer.setTraceLevel(msgLevel);
        this.logRobotPoseEvents = logRobotPoseEvents;
        this.tracePidInfo = tracePidInfo;
        this.verbosePidInfo = verbosePidInfo;
        this.battery = battery;
    }   //setTraceLevel

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param msgLevel specifies the message level.
     * @param logRobotPoseEvents specifies true to log robot pose events, false otherwise.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     * @param verbosePidInfo specifies true to trace verbose PID info, false otherwise.
     */
    public void setTraceLevel(
        TrcDbgTrace.MsgLevel msgLevel, boolean logRobotPoseEvents, boolean tracePidInfo, boolean verbosePidInfo)
    {
        setTraceLevel(msgLevel, logRobotPoseEvents, tracePidInfo, verbosePidInfo, null);
    }   //setTraceLevel

    /**
     * This method sets the beep device and the beep tones so that it can play beeps when motor stalled or if the
     * limit switches are activated/deactivated.
     *
     * @param beepDevice specifies the beep device object.
     * @param beepFrequency specifies the beep frequency.
     * @param beepDuration specifies the beep duration.
     */
    public synchronized void setBeep(TrcTone beepDevice, double beepFrequency, double beepDuration)
    {
        this.beepDevice = beepDevice;
        this.beepFrequency = beepFrequency;
        this.beepDuration = beepDuration;
    }   //setBeep

    /**
     * This method sets the beep device so that it can play beeps at default frequency and duration when motor
     * stalled or if the limit switches are activated/deactivated.
     *
     * @param beepDevice specifies the beep device object.
     */
    public void setBeep(TrcTone beepDevice)
    {
        setBeep(beepDevice, DEF_BEEP_FREQUENCY, DEF_BEEP_DURATION);
    }   //setBeep

    /**
     * This method enables/disables stall detection.
     *
     * @param stallDetectionDelay specifies stall detection start delay in seconds, zero to disable stall detection.
     * @param stallDetectionTimeout specifies stall timeout in seconds which is the minimum elapsed time for the
     *        motor to be motionless to be considered stalled.
     * @param stallErrorRateThreshold specifies the error rate threshold below which it will consider stalling.
     */
    public void setStallDetectionEnabled(
        double stallDetectionDelay, double stallDetectionTimeout, double stallErrorRateThreshold)
    {
        if (xPosPidCtrl != null)
        {
            xPosPidCtrl.setStallDetectionEnabled(stallDetectionDelay, stallDetectionTimeout, stallErrorRateThreshold);
        }
        yPosPidCtrl.setStallDetectionEnabled(stallDetectionDelay, stallDetectionTimeout, stallErrorRateThreshold);
        turnPidCtrl.setStallDetectionEnabled(stallDetectionDelay, stallDetectionTimeout, stallErrorRateThreshold);
    }   //setStallDetectionEnabled

    /**
     * This method enables/disables stall detection.
     *
     * @param enabled specifies true to enable stall detection, false to disable.
     */
    public void setStallDetectionEnabled(boolean enabled)
    {
        if (xPosPidCtrl != null)
        {
            xPosPidCtrl.setStallDetectionEnabled(enabled);
        }
        yPosPidCtrl.setStallDetectionEnabled(enabled);
        turnPidCtrl.setStallDetectionEnabled(enabled);
    }   //setStallDetectionEnabled

    public synchronized void setFastModeEnabled(boolean enabled)
    {
        this.fastModeEnabled = enabled;
    }   //setFastModeEnabled

    /**
     * This method enables/disables incremental turn when running a path segment. Incremental turn is only
     * applicable for holonomic drive base.
     *
     * @param enabled specifies true to enable incremental turn, false to disable.
     */
    public void setIncrementalTurnEnabled(boolean enabled)
    {
        if (xPosPidCtrl != null)
        {
            this.incrementalTurn = enabled;
        }
    }   //setIncrementalTurn

    /**
     * This method enables fixed heading mode. This is especially useful for the robot to track a vision target while
     * following the path. To do this, the Turn PID controller must be vision-based and the heading offset to the
     * vision target is typically zero.
     *
     * @param headingOffset specifies the heading offset supplier.
     */
    public synchronized void enableFixedHeading(TargetHeadingOffset headingOffset)
    {
        if (xPosPidCtrl == null)
        {
            throw new UnsupportedOperationException("FixedHeading is only support on holonomic drive base.");
        }
        this.targetHeadingOffset = headingOffset;
    }   //enableFixedHeading

    /**
     * This method disables fixed heading mode.
     */
    public synchronized void disableFixedHeading()
    {
        this.targetHeadingOffset = null;
    }   //disableFixedHeading

    /**
     * Set both the position tolerance and lookahead radius.
     *
     * @param posTolerance specifies the distance at which the controller will stop itself.
     * @param lookaheadRadius specifies the distance between the robot and next following point.
     */
    public synchronized void setPositionToleranceAndLookaheadRadius(Double posTolerance, Double lookaheadRadius)
    {
        if (posTolerance != null && lookaheadRadius != null && posTolerance >= lookaheadRadius)
        {
            throw new IllegalArgumentException("Position tolerance must be less than lookaheadRadius!");
        }

        if (posTolerance != null)
        {
            this.posTolerance = posTolerance;
        }

        if (lookaheadRadius != null)
        {
            this.lookaheadRadius = lookaheadRadius;
        }
    }   //setPositionToleranceAndLookaheadRadius

    /**
     * Set the position tolerance to end the path. Units need to be consistent.
     *
     * @param posTolerance specifies the distance at which the controller will stop itself.
     */
    public void setPositionTolerance(double posTolerance)
    {
        setPositionToleranceAndLookaheadRadius(posTolerance, null);
    }   //setPositionTolerance

    /**
     * Set the following distance for the pure pursuit controller.
     *
     * @param lookaheadRadius specifies the distance between the robot and next following point.
     */
    public void setlookaheadRadius(double lookaheadRadius)
    {
        setPositionToleranceAndLookaheadRadius(null, lookaheadRadius);
    }   //setlookaheadRadius

    /**
     * Set the turn tolerance for the closed loop control on turning.
     *
     * @param turnTolerance specifies the turn tolerance, in degrees. Should be positive.
     */
    public synchronized void setTurnTolerance(double turnTolerance)
    {
        this.turnTolerance = turnTolerance;
    }   //setTurnTolerance

    /**
     * Sets the pid coefficients for the X position controller. This will work in the middle of an operation as well.
     *
     * @param xPidCoefficients specifies the new PID coefficients for the X position controller.
     */
    public synchronized void setXPositionPidCoefficients(TrcPidController.PidCoefficients xPidCoefficients)
    {
        if (xPosPidCtrl != null)
        {
            xPosPidCtrl.setPidCoefficients(xPidCoefficients);
        }
    }   //setXPositionPidCoefficients

    /**
     * Sets the pid coefficients for the Y position controller. This will work in the middle of an operation as well.
     *
     * @param yPidCoefficients specifies the new PID coefficients for the Y position controller.
     */
    public synchronized void setYPositionPidCoefficients(TrcPidController.PidCoefficients yPidCoefficients)
    {
        yPosPidCtrl.setPidCoefficients(yPidCoefficients);
    }   //setYPositionPidCoefficients

    /**
     * Sets the pid coefficients for both X and Y position controllers. This will work in the middle of an operation as
     * well.
     *
     * @param pidCoefficients specifies the new PID coefficients for both X and Y position controllers.
     */
    public synchronized void setPositionPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        setXPositionPidCoefficients(pidCoefficients);
        setYPositionPidCoefficients(pidCoefficients);
    }   //setPositionPidCoefficients

    /**
     * Sets the pid coefficients for the turn controller. This will work in the middle of an operation as well.
     *
     * @param pidCoefficients specifies the new PID coefficients for the heading controller.
     */
    public synchronized void setTurnPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        turnPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setTurnPidCoefficients

    /**
     * Sets the pid coefficients for the position controller. This will work in the middle of an operation as well.
     * Note that velocity controllers should have an F term as well.
     *
     * @param pidCoefficients specifies the new PIDF coefficients for the velocity controller.
     */
    public synchronized void setVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        velPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setVelocityPidCoefficients

    /**
     * Sets the movement output power limit.
     *
     * @param limit specifies the output power limit for movement (X and Y).
     */
    public synchronized void setMoveOutputLimit(double limit)
    {
        moveOutputLimit = Math.abs(limit);
    }   //setMoveOutputLimit

    /**
     * Sets the rotation output power limit.
     *
     * @param limit specifies the output power limit for rotation.
     */
    public synchronized void setRotOutputLimit(double limit)
    {
        rotOutputLimit = Math.abs(limit);
    }   //setRotOutputLimit

    /**
     * Configure the method of interpolating between waypoints. Methods ending with INV will favor the ending point.
     *
     * @param interpolationType The type of interpolation to use.
     */
    public synchronized void setInterpolationType(InterpolationType interpolationType)
    {
        this.interpolationType = interpolationType == null ? InterpolationType.LINEAR : interpolationType;
    }   //setInterpolationType

    /**
     * This method returns the field position of the target waypoint of the path (i.e. the last waypoint in the path).
     *
     * @return field position of the last waypoint in the path.
     */
    public TrcPose2D getTargetFieldPosition()
    {
        TrcPose2D targetFieldPose = null;

        if (referencePose != null && path != null && path.getSize() > 0)
        {
            targetFieldPose = referencePose.addRelativePose(path.getLastWaypoint().pose);
        }

        return targetFieldPose;
    }   //getTargetFieldPosition

    /**
     * This method returns the target velocity of the current point on the path.
     *
     * @return target velocity of the current point on the path, zero if path is not started or already finished.
     */
    public synchronized double getTargetVelocity()
    {
        return targetVelocity != null? targetVelocity: 0.0;
    }   //getTargetVelocity

    /**
     * This method is called by the Velocity PID controller to get the polar magnitude of the robot's velocity.
     *
     * @return robot's velocity magnitude.
     */
    public synchronized double getRobotVelocity()
    {
        return TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity());
    }   //getRobotVelocity

    /**
     * This method returns the target velocity of the current point on the path.
     *
     * @return target velocity of the current point on the path, zero if path is not started or already finished.
     */
    public double getCurrentRobotVelocity()
    {
        return targetVelocity != null? targetVelocity: 0.0;
    }   //getCurrentTargetVelocity

    /**
     * This method sets the waypoint event handler that gets called when the robot crosses each waypoint. This allows
     * the caller to perform actions when each waypoint is reached. Waypoint handler is cleared when the start method
     * is called. In other words, this method should only be called after the start method is called and the Waypoint
     * event handler is only valid for the path started by the start method.
     *
     * @param handler specifies the waypoint event handler, can be null to clear the event handler.
     */
    public synchronized void setWaypointEventHandler(WaypointEventHandler handler)
    {
        this.waypointEventHandler = handler;
    }   //setWaypointEventHandler

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     * @param path The path to follow. Must start at (0,0).
     * @param event When finished, signal this event.
     * @param timeout Number of seconds after which to cancel this operation. 0.0 for no timeout.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param maxDecel specifies the maximum deceleration if applying trapezoid velocity profile, null if not.
     */
    public synchronized void start(
        String owner, TrcPath path, TrcEvent event, double timeout, Double maxVel, Double maxAccel, Double maxDecel)
    {
        if (path == null || path.getSize() == 0)
        {
            throw new IllegalArgumentException("Path cannot be null or empty!");
        }

        if (driveBase.validateOwnership(owner))
        {
            if (isActive())
            {
                // We successfully validated the ownership but PurePursuit was active. It means somebody was doing
                // PurePursuitDrive with no ownership. Let's cancel it before we take over.
                cancel();
            }

            this.owner = owner;
            this.onFinishedEvent = event;
            if (onFinishedEvent != null)
            {
                onFinishedEvent.clear();
            }
            // Label waypoints with corresponding path indexes.
            TrcWaypoint[] waypoints = path.getAllWaypoints();
            for (int i = 0; i < waypoints.length; i++)
            {
                waypoints[i].index = i;
            }

            TrcPath newPath =
                maxVel != null && maxAccel != null && maxDecel != null?
                    path.trapezoidVelocity(maxVel, maxAccel, maxDecel): path;

            double currTime = TrcTimer.getCurrentTime();
            timedOutTime = timeout == 0.0 ? Double.POSITIVE_INFINITY : currTime + timeout;

            referencePose = driveBase.getFieldPosition();
            pathIndex = 1;
            prevIndex = 0;
            targetVelocity = null;
            firstLookaheadVel = null;
            nearEndTarget = false;

            if (xPosPidCtrl != null)
            {
                xPosPidCtrl.reset();
                xPosPidCtrl.startStallDetection();
            }
            else
            {
                //
                // For non-holonomic drive base, the robot heading must be pointing to the endpoint of the line segment.
                // So, we must ignore the provided startpoint heading and compute our own based on the startpoint heading
                // and the relative angle of the startpoint from the endpoint.
                //
                for (int i = 0; i < newPath.getSize() - 1; i++)
                {
                    TrcWaypoint startPoint = newPath.getWaypoint(i);
                    TrcWaypoint endPoint = newPath.getWaypoint(i + 1);
                    startPoint.pose.angle = Math.toDegrees(Math.atan2(endPoint.pose.x - startPoint.pose.x,
                                                                      endPoint.pose.y - startPoint.pose.y));
                }
            }
            yPosPidCtrl.reset();
            yPosPidCtrl.startStallDetection();
            turnPidCtrl.reset();
            turnPidCtrl.startStallDetection();
            velPidCtrl.reset();

            if (INVERTED_TARGET)
            {
                if (xPosPidCtrl != null)
                {
                    xPosPidCtrl.setTarget(0.0);
                }
                yPosPidCtrl.setTarget(0.0);
                turnPidCtrl.setTarget(driveBase.getHeading());
            }
            this.path = newPath;

            resetError = true;
            driveTaskObj.registerTask(TrcTaskMgr.TaskType.POST_PERIODIC_TASK);

            tracer.traceInfo(instanceName, "Path=" + newPath.toAbsolute(referencePose));
        }
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path The path to follow. Must start at (0,0).
     * @param event When finished, signal this event.
     * @param timeout Number of seconds after which to cancel this operation. 0.0 for no timeout.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param maxDecel specifies the maximum deceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, TrcEvent event, double timeout, Double maxVel, Double maxAccel, Double maxDecel)
    {
        start(null, path, event, timeout, maxVel, maxAccel, maxDecel);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path The path to follow. Must start at (0,0).
     * @param event When finished, signal this event.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param maxDecel specifies the maximum deceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, TrcEvent event, Double maxVel, Double maxAccel, Double maxDecel)
    {
        start(null, path, event, 0.0, maxVel, maxAccel, maxDecel);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path The path to follow. Must start at (0,0).
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param maxDecel specifies the maximum deceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, Double maxVel, Double maxAccel, Double maxDecel)
    {
        start(null, path, null, 0.0, maxVel, maxAccel, maxDecel);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     * @param path The path to follow. Must start at (0,0).
     * @param event When finished, signal this event.
     * @param timeout Number of seconds after which to cancel this operation. 0.0 for no timeout.
     */
    public void start(String owner, TrcPath path, TrcEvent event, double timeout)
    {
        start(owner, path, event, timeout, null, null, null);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path The path to follow. Must start at (0,0).
     * @param event When finished, signal this event.
     * @param timeout Number of seconds after which to cancel this operation. 0.0 for no timeout.
     */
    public void start(TrcPath path, TrcEvent event, double timeout)
    {
        start(null, path, event, timeout, null, null, null);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path The path to follow. Must start at (0,0).
     * @param event When finished, signal this event.
     */
    public void start(TrcPath path, TrcEvent event)
    {
        start(null, path, event, 0.0, null, null, null);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path The path to follow. Must start at (0,0).
     */
    public void start(TrcPath path)
    {
        start(null, path, null, 0.0, null, null, null);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     * @param event When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param maxDecel specifies the maximum deceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        String owner, TrcEvent event, double timeout, boolean incrementalPath, Double maxVel, Double maxAccel,
        Double maxDecel, TrcPose2D... poses)
    {
        TrcPathBuilder pathBuilder = new TrcPathBuilder(driveBase.getFieldPosition(), incrementalPath);

        for (TrcPose2D pose: poses)
        {
            pathBuilder.append(pose);
        }

        start(owner, pathBuilder.toRelativeStartPath(), event, timeout, maxVel, maxAccel, maxDecel);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param event When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxDecel specifies the maximum deceleration if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent event, double timeout, boolean incrementalPath, Double maxVel, Double maxAccel, Double maxDecel,
        TrcPose2D... poses)
    {
        start(null, event, timeout, incrementalPath, maxVel, maxAccel, maxDecel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses read either from the built-in resources
     * or from a file.
     *
     * @param event When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param maxDecel specifies the maximum deceleration if applying trapezoid velocity profile, null if not.
     * @param path specifies the file system path or resource name.
     * @param loadFromResources specifies true if the data is from attached resources, false if from file system.
     */
    public void start(
        TrcEvent event, double timeout, boolean incrementalPath, Double maxVel, Double maxAccel, Double maxDecel,
        String path, boolean loadFromResources)
    {
        start(null, event, timeout, incrementalPath, maxVel, maxAccel, maxDecel,
              TrcPose2D.loadPosesFromCsv(path, loadFromResources));
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param event When finished, signal this event.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param maxDecel specifies the maximum deceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent event, boolean incrementalPath, Double maxVel, Double maxAccel, Double maxDecel, TrcPose2D... poses)
    {
        start(null, event, 0.0, incrementalPath, maxVel, maxAccel, maxDecel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param maxDecel specifies the maximum deceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        boolean incrementalPath, Double maxVel, Double maxAccel, Double maxDecel, TrcPose2D... poses)
    {
        start(null, null, 0.0, incrementalPath, maxVel, maxAccel, maxDecel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     * @param event When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        String owner, TrcEvent event, double timeout, boolean incrementalPath, TrcPose2D... poses)
    {
        start(owner, event, timeout, incrementalPath, null, null, null, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param event When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent event, double timeout, boolean incrementalPath, TrcPose2D... poses)
    {
        start(null, event, timeout, incrementalPath, null, null, null, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param event When finished, signal this event.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent event, boolean incrementalPath, TrcPose2D... poses)
    {
        start(null, event, 0.0, incrementalPath, null, null, null, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(boolean incrementalPath, TrcPose2D... poses)
    {
        start(null, null, 0.0, incrementalPath, null, null, null, poses);
    }   //start

    /**
     * Checks if the robot is currently following a path.
     *
     * @return True if the pure pursuit controller is active, false otherwise.
     */
    public synchronized boolean isActive()
    {
        return driveTaskObj.isRegistered();
    }   //isActive

    /**
     * If the controller is currently active, cancel the path following operation. Otherwise, do nothing.
     * If there is an event to signal, mark it as cancelled.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     */
    public synchronized void cancel(String owner)
    {
        if (isActive() && driveBase.validateOwnership(owner))
        {
            stop();

            if (onFinishedEvent != null)
            {
                onFinishedEvent.cancel();
                onFinishedEvent = null;
            }
            //
            // Either the path is done or canceled, call the event handler one last time with index -1 and be done.
            //
            if (waypointEventHandler != null)
            {
                waypointEventHandler.waypointEvent(-1, null);
                waypointEventHandler = null;
            }
        }
    }   //cancel

    /**
     * If the controller is currently active, cancel the path following operation. Otherwise, do nothing.
     * If there is an event to signal, mark it as cancelled.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * Stops PurePursuit drive.
     */
    private synchronized void stop()
    {
        driveTaskObj.unregisterTask();
        driveBase.stop(owner);
        path = null;
        owner = null;
    }   //stop

    /**
     * This method returns the pure pursuit path.
     *
     * @return path.
     */
    public synchronized TrcPath getPath()
    {
        return path;
    }   //getPath

    /**
     * This method is called by xPosPidCtrl only in INVERTED_TARGET mode for getting the X distance to target.
     * @return x distance to target.
     */
    private synchronized double getXPosition()
    {
        return targetPose != null? targetPose.x: 0.0;
    }   //getXPosition

    /**
     * This method is called by yPosPidCtrl only in INVERTED_TARGET mode for getting the Y distance to target.
     * @return y distance to target.
     */
    private synchronized double getYPosition()
    {
        return targetPose != null? targetPose.y: 0.0;
    }   //getYPosition

    /**
     * This task is called periodically to calculate the next target point on the path. The next target point on
     * the path has a distance of followDistance from the current robot position intersecting with the path segment
     * towards the end of the endpoint of the path segment.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private synchronized void driveTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        if (path != null)
        {
            TrcPose2D robotPose = driveBase.getPositionRelativeTo(referencePose, true);
            TrcWaypoint targetPoint = getFollowingPoint(robotPose);
            TrcWaypoint segmentStart = path.getWaypoint(pathIndex - 1);
            TrcWaypoint segmentEnd = path.getWaypoint(pathIndex);
            TrcWaypoint prevSegmentStart = pathIndex - 1 > 0? path.getWaypoint(pathIndex - 2): null;    //TODO:remove
            double segmentLength = segmentEnd.distanceTo(segmentStart);
            boolean lastSegment = pathIndex == path.getSize() - 1;

            // TODO:remove
            if (firstLookaheadVel == null)
            {
                firstLookaheadVel = targetPoint.velocity;
            }

            tracer.traceDebug(
                instanceName,
                "***** robotPose=%s, lookaheadPose=%s, index=%d/%d, segStart=%s(vel=%f), segEnd=%s(vel=%f), segLen =%f",
                robotPose, targetPoint.pose, pathIndex, path.getSize(), segmentStart.pose, segmentStart.velocity,
                segmentEnd.pose, segmentEnd.velocity, segmentLength);
            targetPose = getTargetPose(robotPose, targetPoint, segmentStart, segmentEnd);
            // TODO:remove targetPoint, prevSegmentStart
            targetVelocity = getTargetVelocity(robotPose, targetPoint, prevSegmentStart, segmentStart, segmentEnd);
            if (!INVERTED_TARGET)
            {
                // We only initialize the PID controller error state at the beginning. Once the path following has
                // started, all subsequent setTarget calls should not re-initialize PID controller error states
                // because we are just updating the target and do not really want to destroy totalError or previous
                // error that will screw up the subsequent calculate() calls where it needs the previous error states
                // to compute the I and D terms correctly.
                if (xPosPidCtrl != null)
                {
                    xPosPidCtrl.setTarget(targetPose.x, resetError);
                }
                yPosPidCtrl.setTarget(targetPose.y, resetError);
            }
            turnPidCtrl.setTarget(targetPose.angle, warpSpace, resetError);
            velPidCtrl.setTarget(targetVelocity, resetError);
            resetError = false;

            double xPosPower = xPosPidCtrl != null? xPosPidCtrl.getOutput(): 0.0;
            double yPosPower = yPosPidCtrl.getOutput();
            double turnPower = turnPidCtrl.getOutput();
            double velPower = segmentLength > 0.0? velPidCtrl.getOutput(): 0.0;
            double theta = Math.atan2(targetPose.x, targetPose.y);
            xPosPower = xPosPidCtrl == null? 0.0:
                TrcUtil.clipRange(xPosPower + velPower * Math.sin(theta), -moveOutputLimit, moveOutputLimit);
            yPosPower = TrcUtil.clipRange(yPosPower + velPower * Math.cos(theta), -moveOutputLimit, moveOutputLimit);
            turnPower = TrcUtil.clipRange(turnPower, -rotOutputLimit, rotOutputLimit);

            tracer.traceDebug(
                instanceName,
                "Errors(x/y/turn/vel)=%f/%f/%f/%f, Powers(x/y/turn/vel)=%f/%f/%f/%f, theta=%f",
                xPosPidCtrl != null? xPosPidCtrl.getError(): 0.0, yPosPidCtrl.getError(), turnPidCtrl.getError(),
                velPidCtrl.getError(), xPosPower, yPosPower, turnPower, velPower, Math.toDegrees(theta));

            // If we have timed out or finished, stop the operation.
            double currTime = TrcTimer.getCurrentTime();
            boolean timedOut = currTime >= timedOutTime;

            stalled = (xPosPidCtrl == null || xPosPidCtrl.isStalled()) &&
                      yPosPidCtrl.isStalled() && turnPidCtrl.isStalled();
            boolean posOnTarget =
                (xPosPidCtrl == null || xPosPidCtrl.isOnTarget(posTolerance)) && yPosPidCtrl.isOnTarget(posTolerance);
            boolean headingOnTarget = turnPidCtrl.isOnTarget(turnTolerance, 0.0);

            tracer.traceInfo(
                instanceName,
                "OnTarget(pos/turn)=%s/%s, timedOut=%s, stalled=%s", posOnTarget, headingOnTarget, timedOut, stalled);
            if (beepDevice != null && (stalled || timedOut))
            {
                beepDevice.playTone(beepFrequency, beepDuration);
            }

            if (timedOut || lastSegment && (stalled || posOnTarget && headingOnTarget))
            {
                tracer.traceInfo(
                    instanceName,
                    "Done: index=" + pathIndex + "/" + path.getSize() +
                    ", stalled=" + stalled +
                    ", timeout=" + timedOut +
                    ", posOnTarget=" + posOnTarget +
                    ",headingOnTarget=" + headingOnTarget +
                    ",robotPose=" + driveBase.getFieldPosition());
                if (tracePidInfo)
                {
                    if (xPosPidCtrl != null) xPosPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
                    yPosPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
                    turnPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
                    velPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
                }

                stop();

                if (onFinishedEvent != null)
                {
                    tracer.traceInfo(instanceName, "Signal completion event " + onFinishedEvent + ".");
                    onFinishedEvent.signal();
                    onFinishedEvent = null;
                }

                if (xPosPidCtrl != null) xPosPidCtrl.endStallDetection();
                yPosPidCtrl.endStallDetection();
                turnPidCtrl.endStallDetection();
            }
            else if (xPosPidCtrl != null)
            {
                driveBase.holonomicDrive(owner, xPosPower, yPosPower, turnPower);
            }
            else
            {
                driveBase.arcadeDrive(owner, yPosPower, turnPower, false);
            }

            if (logRobotPoseEvents)
            {
                tracer.logEvent(
                    instanceName, "RobotPose",
                    "AbsPose=\"" + driveBase.getFieldPosition() +
                    "\" AbsTarget=\"" + referencePose.addRelativePose(targetPoint.pose) +
                    "\" Delta=\"" + targetPose + "\"");
            }

            if (tracePidInfo)
            {
                if (xPosPidCtrl != null) xPosPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
                yPosPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
                turnPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
                velPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
            }
        }
    }   //driveTask

    private TrcPose2D getTargetPose(
        TrcPose2D robotPose, TrcWaypoint lookaheadPoint, TrcWaypoint segmentStart, TrcWaypoint segmentEnd)
    {
        TrcPose2D targetPose;
        double distanceToStart = robotPose.distanceTo(segmentStart.pose);
        double distanceToEnd = robotPose.distanceTo(segmentEnd.pose);
        double targetHeading;

        targetPose = lookaheadPoint.pose.relativeTo(robotPose, true);
        if (targetHeadingOffset != null)
        {
            Double headingOffset = targetHeadingOffset.getOffset();
            if (headingOffset != null)
            {
                targetHeading = driveBase.getHeading() + headingOffset;
            }
            else
            {
                targetHeading = targetPose.angle + referencePose.angle + robotPose.angle;
            }
        }
        else
        {
            targetHeading = targetPose.angle + referencePose.angle + robotPose.angle;
        }

//        if (fastMode &&
//            (segmentStart.velocity == 0.0 || distanceToStart > lookaheadRadius && distanceToEnd > lookaheadRadius))
        if (fastMode && distanceToStart > lookaheadRadius && distanceToEnd > lookaheadRadius)
        {
            targetPose = segmentEnd.pose.relativeTo(robotPose, true);
        }
        // Use interpolated heading instead of the end point heading.
        targetPose.angle = targetHeading;
        tracer.traceDebug(
            instanceName,
            "TargetPose=%s (fastMode=%s, distToStart=%f, distToEnd=%f)",
            targetPose, fastMode, distanceToStart, distanceToEnd);

        return targetPose;
    }   //getTargetPose

    private double getTargetVelocity(
        TrcPose2D robotPose, TrcWaypoint lookaheadPoint, TrcWaypoint prevSegmentStart, TrcWaypoint segmentStart,
        TrcWaypoint segmentEnd)
    {
        double targetVel;
        boolean decelerating = segmentStart.velocity > segmentEnd.velocity;

        if (segmentEnd.distanceTo(segmentStart) == 0.0)
        {
            // Turn only.
            targetVel = 0.0;
            tracer.traceDebug(instanceName, "TargetVel=0.0 (Turn-only)");
        }
        else if (useLookaheadVelUntilDecel && !decelerating)
        {
            // Algorithm 2.
            targetVel = lookaheadPoint.velocity;
            tracer.traceDebug(instanceName, "TargetVel=%f (LookaheadVelUntilDecel)", targetVel);
        }
        else if (constantVelInLookahead)
        {
            double distanceToStart = robotPose.distanceTo(segmentStart.pose);
            double distanceToEnd = robotPose.distanceTo(segmentEnd.pose);
            boolean robotNearStart = false, robotNearEnd = false;

            if (nearEndTarget)
            {
                targetVel = segmentEnd.velocity;
            }
            else if (distanceToEnd <= lookaheadRadius)
            {
                targetVel = segmentEnd.velocity;
                robotNearEnd = true;
                if (segmentEnd.velocity == 0.0)
                {
                    nearEndTarget = true;
                }
            }
            else if (distanceToStart <= lookaheadRadius)
            {
                targetVel = segmentStart.velocity != 0.0? segmentStart.velocity:
                    interpolate(
                        segmentStart.velocity, segmentEnd.velocity, distanceToStart/(distanceToStart + distanceToEnd));
                robotNearStart = true;
            }
            else
            {
                targetVel = interpolate(
                    segmentStart.velocity, segmentEnd.velocity, distanceToStart/(distanceToStart + distanceToEnd));
            }
            tracer.traceDebug(
                instanceName, "TargetVel=%f (NearStart=%s, NearEnd=%s, robotVel=%f)",
                targetVel, robotNearStart, robotNearEnd, getRobotVelocity());
        }
        else
        {
            TrcWaypoint segStart = segmentStart, segEnd = segmentEnd;
            double distanceToStart = robotPose.distanceTo(segmentStart.pose);
            TrcPose2D projectedPose = orthogonalProjectedPointOnLine(segmentStart.pose, segmentEnd.pose, robotPose);

            if (distanceToStart < lookaheadRadius)
            {
                if (prevSegmentStart == null)
                {
                    // Algorithm 1.
                    // On first path segment and within lookahead circle of the starting point.
                    targetVel = firstLookaheadVel;
                    tracer.traceDebug(instanceName, "TargetVel=%f (Algorithm 1: first LookaheadVel)", targetVel);
                }
                else
                {
                    // Entering the next path segment.
                    double distanceToEnd = robotPose.distanceTo(segmentEnd.pose);
                    tracer.traceDebug(
                        instanceName,
                        "[%d] robotPose=%s, prevSegStart=%s, segStart=%s, segEnd=%s, projPose=%s, distToStart=%f, " +
                        "distToEnd=%f",
                        pathIndex, robotPose, prevSegmentStart.pose, segmentStart.pose, segmentEnd.pose,
                        projectedPose, distanceToStart, distanceToEnd);

                    if (projectedPose.x < Math.min(segStart.pose.x, segEnd.pose.x) ||
                        projectedPose.x > Math.max(segStart.pose.x, segEnd.pose.x))
                    {
                        segStart = prevSegmentStart;
                        segEnd = segmentStart;
                        projectedPose = orthogonalProjectedPointOnLine(segStart.pose, segEnd.pose, robotPose);
                    }
                }
            }

            double segLength = segEnd.distanceTo(segStart);
            targetVel = segLength > 0.0?
                interpolate(segStart.velocity, segEnd.velocity,
                            TrcUtil.clipRange(projectedPose.distanceTo(segStart.pose)/segLength, 0.0, 1.0)): 0.0;
            tracer.traceDebug(
                instanceName,
                "segStart=%s(%f), segEnd=%s(%f), segLen=%f, projPose=%s(%f), robotVel=%f",
                segStart.pose, segStart.velocity, segEnd.pose, segEnd.velocity, segLength, projectedPose, targetVel,
                getRobotVelocity());
        }

        return targetVel;
    }   //getTargetVelocity

    /**
     * This method orthogonally projects the given point onto the given line segment and returns the projected point.
     * <a href="https://math.stackexchange.com/questions/62633/orthogonal-projection-of-a-point-onto-a-line">...</a>
     * <p>
     * m = (Y1 - Y0)/(X1 - X0)
     * <p>
     * -1/m = (Y - Yp)/(X - Xp)
     * => m(Y - Yp) = Xp - X
     * => mY - mYp = Xp - X
     * => Xp + mYp = X + mY
     * <p>
     * Yp = mXp + b
     * => Xp + m(mXp + b) = X + mY
     * => Xp + m^2*Xp + mb = X + mY
     * => Xp(1 + m^2) + mb = X + mY
     * => Xp = (X + mY - mb)/(1 + m^2)
     * => Xp = (X + m(Y - b))/(1 + m^2)
     * <p>
     * Yp = m(X + mY - mb)/(1 + m^2) + b
     * => Yp = (mX + m^2*Y - m^2*b)/(1 + m^2) + b
     * => Yp = (mX + m^2*Y - m^2*b + b + m^2*b)/(1 + m^2)
     * => Yp = (mX + m^2*Y + b)/(1 + m^2)
     * => Yp = (m*(X + mY) + b)/(1 + m^2)
     *
     * @param lineStart specifies the starting point of the line segment.
     * @param lineEnd specifies the ending point of the line segment.
     * @param point specifies the point to be projected onto the line segment.
     * @return projected point.
     */
    private TrcPose2D orthogonalProjectedPointOnLine(TrcPose2D lineStart, TrcPose2D lineEnd, TrcPose2D point)
    {
        double deltaX = lineEnd.x - lineStart.x;
        if (deltaX != 0.0)
        {
            double m = (lineEnd.y - lineStart.y)/deltaX;
            double b = lineStart.y - m*lineStart.x;
            double den = 1.0 + m*m;
            return new TrcPose2D((point.x + m*(point.y - b))/den, (m*(point.x + m*point.y) + b)/den, point.angle);
        }
        else
        {
            return new TrcPose2D(lineStart.x, point.y, point.angle);
        }
    }   //orthogonalProjectedPointOnLine

    /**
     * Returns a weighted value between given values.
     *
     * @param startValue specifies the start value.
     * @param endValue specifies the end value.
     * @param weight specifies the weight between the values.
     * @return weighted value between the given values.
     */
    private double interpolate(double startValue, double endValue, double weight)
    {
        switch (interpolationType)
        {
            case LINEAR:
            case QUADRATIC:
            case CUBIC:
            case QUARTIC:
                weight = Math.pow(weight, interpolationType.value);
                break;

            case QUADRATIC_INV:
            case CUBIC_INV:
            case QUARTIC_INV:
                weight = Math.pow(weight, 1.0 / interpolationType.value);
                break;
        }
        // This should work for interpolate as well as extrapolate.
        return startValue + weight*(endValue - startValue);
    }   //interpolate

    /**
     * Interpolates a waypoint that's weighted between two given waypoints.
     *
     * @param point1 specifies the start point of the path segment.
     * @param point2 specifies the end point of the path segment.
     * @param weight specifies the weight between the two provided points.
     * @param robotPose specifies the robot's position, set to null for holonomic drivebase.
     * @return weighted interpolated waypoint.
     */
    private TrcWaypoint interpolate(TrcWaypoint point1, TrcWaypoint point2, double weight, TrcPose2D robotPose)
    {
        double timestep = interpolate(point1.timeStep, point2.timeStep, weight);
        double x = interpolate(point1.pose.x, point2.pose.x, weight);
        double y = interpolate(point1.pose.y, point2.pose.y, weight);
        double position = interpolate(point1.encoderPosition, point2.encoderPosition, weight);
        double velocity = interpolate(point1.velocity, point2.velocity, weight);
        double acceleration = interpolate(point1.acceleration, point2.acceleration, weight);
        double jerk = interpolate(point1.jerk, point2.jerk, weight);

        double heading;
        double turningRadius = lookaheadRadius + posTolerance;
        if (robotPose == null || robotPose.distanceTo(point2.pose) <= turningRadius)
        {
            if (robotPose != null)
            {
                // For non-holonomic drivebase and the end-waypoint is within the robot's lookahead circle +
                // posTolerance, we will interpolate the heading weight differently than holonomic drivebase.
                // The heading weight is the percentage distance of the robot position to the end-waypoint over
                // lookahead radius.
                weight = 1 - point2.pose.distanceTo(point1.pose)*(1 - weight)/turningRadius;
            }
            heading = interpolate(
                point1.pose.angle, warpSpace.getOptimizedTarget(point2.pose.angle, point1.pose.angle), weight);
        }
        else
        {
            // For non-holonomic drivebase, maintain the robot heading pointing to the end-waypoint unless the
            // end-waypoint is within the robot's lookahead circle.
            TrcPose2D endpointPose = point2.pose.clone();
            endpointPose.angle = point1.pose.angle;
            heading = point1.pose.angle + endpointPose.relativeTo(robotPose).angle;
        }

        return new TrcWaypoint(timestep, new TrcPose2D(x, y, heading), position, velocity, acceleration, jerk);
    }   //interpolate

    /**
     * This method calculates the waypoint on the path segment that intersects the robot's lookahead circle that is
     * closest to the end point of the path segment. The algorithm is based on this article:
     * <a href="https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm">...</a>
     *
     * @param startWaypoint specifies the start point of the path segment.
     * @param endWaypoint specifies the end point of the path segment.
     * @param robotPose specifies the robot's position.
     * @return calculated waypoint.
     */
    private TrcWaypoint getFollowingPointOnSegment(
        TrcWaypoint startWaypoint, TrcWaypoint endWaypoint, TrcPose2D robotPose)
    {
        if (fastModeEnabled && robotPose.distanceTo(endWaypoint.pose) > lookaheadRadius)
        {
            tracer.traceDebug(
                instanceName, "pathIndex=%d, startPose=%s, endPose=%s",
                pathIndex, startWaypoint.getPositionPose(), endWaypoint.getPositionPose());
            return interpolate(
                startWaypoint, endWaypoint, 1.0, !incrementalTurn? robotPose: null);
        }
        else
        {
            // Find intersection of path segment with the lookahead circle of the robot.
            RealVector startVector = startWaypoint.getPositionPose().toPosVector();
            RealVector endVector = endWaypoint.getPositionPose().toPosVector();
            RealVector robotVector = robotPose.toPosVector();

            RealVector startToEnd = endVector.subtract(startVector);
            RealVector robotToStart = startVector.subtract(robotVector);
            // Solve quadratic formula
            double a = startToEnd.dotProduct(startToEnd);
            double b = 2 * robotToStart.dotProduct(startToEnd);
            double c = robotToStart.dotProduct(robotToStart) - lookaheadRadius * lookaheadRadius;

            double discriminant = b * b - 4 * a * c;
            if (discriminant < 0 || a == 0.0)
            {
                // No valid intersection or end waypoint has the same location as the start waypoint.
                // (i.e. same x and y but could have different angles).
                tracer.traceDebug(instanceName, "No valid intersection.");
                return null;
            }
//            else if (a == 0.0)
//            {
//                // The end waypoint has the same location as the start waypoint (i.e. same x and y but could have
//                // different angles), just return the end waypoint to make it a turn-only drive.
//                return endWaypoint;
//            }
            else
            {
                // line is a parametric equation, where t=0 is start waypoint, t=1 is end waypoint of the line segment.
                discriminant = Math.sqrt(discriminant);
                // t1 and t2 represent the relative positions of the intersection points on the line segment. If they are
                // in the range of 0.0 and 1.0, they are on the line segment. Otherwise, the intersection points are
                // outside of the line segment. If the relative position is towards 0.0, it is closer to the start
                // waypoint of the line segment. If the relative position is towards 1.0, it is closer to the end
                // waypoint of the line segment.
                //
                // t represents the furthest intersection point (the one closest to the end waypoint of the line segment).
                double t1 = (-b - discriminant)/(2*a);
                double t2 = (-b + discriminant)/(2*a);
                double t = Math.max(t1, t2);

                if (!TrcUtil.inRange(t, 0.0, 1.0))
                {
                    //
                    // The furthest intersection point is not on the line segment, so skip this segment.
                    //
                    tracer.traceDebug(
                        instanceName, "Intersection not on line segment t1=%f, t2=%f, t=%f, stalled=%s",
                        t1, t2, t, stalled);
                    return null;
                }

                TrcWaypoint interpolated =
                    interpolate(startWaypoint, endWaypoint, t, xPosPidCtrl == null? robotPose: null);
                tracer.traceDebug(
                    instanceName,
                    "InterpolatedPoint=%s, startPoint=%s, endPoint=%s",
                    interpolated.getPositionPose(), startWaypoint.getPositionPose(), endWaypoint.getPositionPose());

                return interpolated;
            }
        }
    }   //getFollowingPointOnSegment

    /**
     * Determines the next target point for Pure Pursuit Drive to follow.
     *
     * @param robotPose specifies the robot's location.
     * @return next target point for the robot to follow.
     */
    private TrcWaypoint getFollowingPoint(TrcPose2D robotPose)
    {
        TrcWaypoint lastWaypoint = path.getLastWaypoint();
        if (robotPose.distanceTo(lastWaypoint.pose) < lookaheadRadius)
        {
            pathIndex = path.getSize() - 1;
            return lastWaypoint;
        }
        // Find the next segment that intersects with the lookahead circle of the robot.
        // If there are tiny segments that are completely within the lookahead circle, we will skip them all.
        for (int i = Math.max(pathIndex, 1); i < path.getSize(); i++)
        {
            TrcWaypoint segmentStart = path.getWaypoint(i - 1);
            TrcWaypoint segmentEnd = path.getWaypoint(i);
            // If there is a valid intersection, return it.
            TrcWaypoint interpolated = getFollowingPointOnSegment(segmentStart, segmentEnd, robotPose);
            if (interpolated != null)
            {
                if (pathIndex != i)
                {
                    // We are moving to the next waypoint.
                    if (waypointEventHandler != null && segmentStart.index != -1)
                    {
                        waypointEventHandler.waypointEvent(segmentStart.index, segmentStart);
                    }
                }
                tracer.traceDebug(
                    instanceName, "Segment[%d:%s->%d:%s] Index=%d->%d, Target=%s",
                    i - 1, segmentStart.pose, i, segmentEnd.pose, pathIndex, i, interpolated);
                pathIndex = i;
                return interpolated;
            }
            else if (stalled)
            {
                tracer.traceInfo(instanceName, "Segment " + i + " is stalled, moving to the next segment.");
                pathIndex = i;
            }
        }
        // Found no intersection. The robot must be off-path. Just proceed to the immediate next waypoint.
        tracer.traceInfo(instanceName, "No intersection found, proceed to the immediate next waypoint " + pathIndex);
        return path.getWaypoint(pathIndex);
    }   //getFollowingPoint

}   //class TrcPurePursuitDrive
