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

import trclib.robotcore.TrcPidController;
import trclib.driverio.TrcTone;
import trclib.dataprocessor.TrcUtil;
import trclib.dataprocessor.TrcWarpSpace;
import trclib.drivebase.TrcDriveBase;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.sensor.TrcRobotBattery;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent Pure Pursuit drive for holonomic or non-holonomic robots.
 * Essentially, a pure pursuit drive navigates the robot to chase a point along the path. The point to chase is
 * chosen by intersecting a proximity circle centered on the robot with a specific radius with the path, and chasing
 * the "furthest" intersection. The smaller the radius is, the more "tightly" the robot will follow a path, but it
 * will be more prone to oscillation and sharp turns. A larger radius will tend to smooth out turns and corners. Note
 * that the error tolerance must be less than the proximity radius, so choose them accordingly.
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
 * smoothing, which we don't do, since a nonzero proximity radius will naturally smooth it anyway.
 */
public class TrcPurePursuitDrive
{
    private static final boolean INVERTED_TARGET = false;

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
    private volatile double proximityRadius;    // Volatile so it can be changed at runtime
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
    private TrcPose2D relativeTargetPose;
    private boolean fastModeEnabled = false;
    private boolean resetError = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the reference to the drive base.
     * @param proximityRadius specifies the distance between the robot and next following point.
     * @param posTolerance specifies the position tolerance.
     * @param turnTolerance specifies the turn tolerance.
     * @param xPidCtrl specifies the position PID controller for X.
     * @param yPidCtrl specifies the position PID controller for Y.
     * @param turnPidCtrl specifies the turn PID controller.
     * @param velPidCtrl specifies the velocity PID controller.
     */
    public void commonInit(
        String instanceName, TrcDriveBase driveBase, double proximityRadius, double posTolerance, double turnTolerance,
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

        setPositionToleranceAndProximityRadius(posTolerance, proximityRadius);
        this.turnTolerance = turnTolerance;

        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);
        driveTaskObj = TrcTaskMgr.createTask(instanceName + ".driveTask", this::driveTask);
    }   //commonInit

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the reference to the drive base.
     * @param proximityRadius specifies the distance between the robot and next following point.
     * @param posTolerance specifies the position tolerance.
     * @param turnTolerance specifies the turn tolerance.
     * @param xPidCtrl specifies the position PID controller for X.
     * @param yPidCtrl specifies the position PID controller for Y.
     * @param turnPidCtrl specifies the turn PID controller.
     * @param velPidCtrl specifies the velocity PID controller.
     */
    public TrcPurePursuitDrive(
        String instanceName, TrcDriveBase driveBase, double proximityRadius, double posTolerance, double turnTolerance,
        TrcPidController xPidCtrl, TrcPidController yPidCtrl, TrcPidController turnPidCtrl,
        TrcPidController velPidCtrl)
    {
        if (xPidCtrl != null && !driveBase.supportsHolonomicDrive())
        {
            throw new IllegalArgumentException(
                "xPosPidCtrl is provided but drive base does not support holonomic drive!");
        }

        commonInit(
            instanceName, driveBase, proximityRadius, posTolerance, turnTolerance, xPidCtrl, yPidCtrl, turnPidCtrl,
            velPidCtrl);
    }   //TrcPurePursuitDrive

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the reference to the drive base.
     * @param proximityRadius specifies the distance between the robot and next following point.
     * @param posTolerance specifies the position tolerance.
     * @param turnTolerance specifies the turn tolerance.
     * @param xPosPidCoeff specifies the position PID coefficients for X.
     * @param yPosPidCoeff specifies the position PID coefficients for Y.
     * @param turnPidCoeff specifies the turn PID coefficients.
     * @param velPidCoeff specifies the velocity PID coefficients.
     */
    public TrcPurePursuitDrive(
        String instanceName, TrcDriveBase driveBase, double proximityRadius, double posTolerance, double turnTolerance,
        TrcPidController.PidCoefficients xPosPidCoeff, TrcPidController.PidCoefficients yPosPidCoeff,
        TrcPidController.PidCoefficients turnPidCoeff, TrcPidController.PidCoefficients velPidCoeff)
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
        velPidCtrl = new TrcPidController(instanceName + ".velPid", velPidCoeff, this::getVelocityInput);

        commonInit(
            instanceName, driveBase, proximityRadius, posTolerance, turnTolerance, xPidCtrl, yPidCtrl, turnPidCtrl,
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
     * Set both the position tolerance and proximity radius.
     *
     * @param posTolerance specifies the distance at which the controller will stop itself.
     * @param proximityRadius specifies the distance between the robot and next following point.
     */
    public synchronized void setPositionToleranceAndProximityRadius(Double posTolerance, Double proximityRadius)
    {
        if (posTolerance != null && proximityRadius != null && posTolerance >= proximityRadius)
        {
            throw new IllegalArgumentException("Position tolerance must be less than proximityRadius!");
        }

        if (posTolerance != null)
        {
            this.posTolerance = posTolerance;
        }

        if (proximityRadius != null)
        {
            this.proximityRadius = proximityRadius;
        }
    }   //setPositionToleranceAndProximityRadius

    /**
     * Set the position tolerance to end the path. Units need to be consistent.
     *
     * @param posTolerance specifies the distance at which the controller will stop itself.
     */
    public void setPositionTolerance(double posTolerance)
    {
        setPositionToleranceAndProximityRadius(posTolerance, null);
    }   //setPositionTolerance

    /**
     * Set the following distance for the pure pursuit controller.
     *
     * @param proximityRadius specifies the distance between the robot and next following point.
     */
    public void setProximityRadius(double proximityRadius)
    {
        setPositionToleranceAndProximityRadius(null, proximityRadius);
    }   //setProximityRadius

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
        TrcPose2D targetPose = null;

        if (referencePose != null && path != null && path.getSize() > 0)
        {
            targetPose = referencePose.addRelativePose(path.getLastWaypoint().pose);
        }

        return targetPose;
    }   //getTargetFieldPosition

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
     */
    public synchronized void start(
        String owner, TrcPath path, TrcEvent event, double timeout, Double maxVel, Double maxAccel)
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

            this.path = maxVel != null && maxAccel != null? path.trapezoidVelocity(maxVel, maxAccel): path;

            double currTime = TrcTimer.getCurrentTime();
            timedOutTime = timeout == 0.0 ? Double.POSITIVE_INFINITY : currTime + timeout;

            referencePose = driveBase.getFieldPosition();
            pathIndex = 1;

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
                for (int i = 0; i < path.getSize() - 1; i++)
                {
                    TrcWaypoint startPoint = path.getWaypoint(i);
                    TrcWaypoint endPoint = path.getWaypoint(i + 1);
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

            resetError = true;
            driveTaskObj.registerTask(TrcTaskMgr.TaskType.POST_PERIODIC_TASK);

            tracer.traceInfo(instanceName, "Path=" + path.toAbsolute(referencePose));
        }
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param event            When finished, signal this event.
     * @param timeout         Number of seconds after which to cancel this operation. 0.0 for no timeout.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, TrcEvent event, double timeout, Double maxVel, Double maxAccel)
    {
        start(null, path, event, timeout, maxVel, maxAccel);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param event            When finished, signal this event.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, TrcEvent event, Double maxVel, Double maxAccel)
    {
        start(null, path, event, 0.0, maxVel, maxAccel);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, Double maxVel, Double maxAccel)
    {
        start(null, path, null, 0.0, maxVel, maxAccel);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param owner           specifies the ID string of the caller requesting exclusive access.
     * @param path            The path to follow. Must start at (0,0).
     * @param event            When finished, signal this event.
     * @param timeout         Number of seconds after which to cancel this operation. 0.0 for no timeout.
     */
    public void start(String owner, TrcPath path, TrcEvent event, double timeout)
    {
        start(owner, path, event, timeout, null, null);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param event            When finished, signal this event.
     * @param timeout         Number of seconds after which to cancel this operation. 0.0 for no timeout.
     */
    public void start(TrcPath path, TrcEvent event, double timeout)
    {
        start(null, path, event, timeout, null, null);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param event            When finished, signal this event.
     */
    public void start(TrcPath path, TrcEvent event)
    {
        start(null, path, event, 0.0, null, null);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path The path to follow. Must start at (0,0).
     */
    public void start(TrcPath path)
    {
        start(null, path, null, 0.0, null, null);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param owner           specifies the ID string of the caller requesting exclusive access.
     * @param event            When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        String owner, TrcEvent event, double timeout, TrcPose2D startingPose, boolean incrementalPath,
        Double maxVel, Double maxAccel, TrcPose2D... poses)
    {
        TrcPathBuilder pathBuilder = new TrcPathBuilder(startingPose, incrementalPath);

        for (TrcPose2D pose: poses)
        {
            pathBuilder.append(pose);
        }

        start(owner, pathBuilder.toRelativeStartPath(), event, timeout, maxVel, maxAccel);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param event            When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent event, double timeout, TrcPose2D startingPose, boolean incrementalPath, Double maxVel, Double maxAccel,
        TrcPose2D... poses)
    {
        start(null, event, timeout, startingPose, incrementalPath, maxVel, maxAccel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses read either from the built-in resources
     * or from a file.
     *
     * @param event            When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param path specifies the file system path or resource name.
     * @param loadFromResources specifies true if the data is from attached resources, false if from file system.
     */
    public void start(
        TrcEvent event, double timeout, TrcPose2D startingPose, boolean incrementalPath, Double maxVel, Double maxAccel,
        String path, boolean loadFromResources)
    {
        start(null, event, timeout, startingPose, incrementalPath, maxVel, maxAccel,
              TrcPose2D.loadPosesFromCsv(path, loadFromResources));
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param event            When finished, signal this event.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent event, TrcPose2D startingPose, boolean incrementalPath, Double maxVel, Double maxAccel,
        TrcPose2D... poses)
    {
        start(null, event, 0.0, startingPose, incrementalPath, maxVel, maxAccel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcPose2D startingPose, boolean incrementalPath, Double maxVel, Double maxAccel, TrcPose2D... poses)
    {
        start(null, null, 0.0, startingPose, incrementalPath, maxVel, maxAccel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param owner           specifies the ID string of the caller requesting exclusive access.
     * @param event            When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        String owner, TrcEvent event, double timeout, TrcPose2D startingPose, boolean incrementalPath,
        TrcPose2D... poses)
    {
        start(owner, event, timeout, startingPose, incrementalPath, null, null, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param event            When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent event, double timeout, TrcPose2D startingPose, boolean incrementalPath, TrcPose2D... poses)
    {
        start(null, event, timeout, startingPose, incrementalPath, null, null, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param event            When finished, signal this event.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent event, TrcPose2D startingPose, boolean incrementalPath, TrcPose2D... poses)
    {
        start(null, event, 0.0, startingPose, incrementalPath, null, null, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(TrcPose2D startingPose, boolean incrementalPath, TrcPose2D... poses)
    {
        start(null, null, 0.0, startingPose, incrementalPath, null, null, poses);
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
        return relativeTargetPose != null? relativeTargetPose.x: 0.0;
    }   //getXPosition

    /**
     * This method is called by yPosPidCtrl only in INVERTED_TARGET mode for getting the Y distance to target.
     * @return y distance to target.
     */
    private synchronized double getYPosition()
    {
        return relativeTargetPose != null? relativeTargetPose.y: 0.0;
    }   //getYPosition

    /**
     * This method is called by the Velocity PID controller to get the polar magnitude of the robot's velocity.
     *
     * @return robot's velocity magnitude.
     */
    private synchronized double getVelocityInput()
    {
        return TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity());
    }   //getVelocityInput

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
        TrcPose2D robotPose = driveBase.getPositionRelativeTo(referencePose, true);
        TrcWaypoint targetPoint = getFollowingPoint(robotPose);

        if (path != null)
        {
            relativeTargetPose = targetPoint.pose.relativeTo(robotPose, true);
            boolean lastSegment = pathIndex == path.getSize() - 1;

            if (!INVERTED_TARGET)
            {
                //
                // We only initialize the PID controller error state at the beginning. Once the path following has started,
                // all subsequent setTarget calls should not re-initialize PID controller error states because we are just
                // updating the target and do not really want to destroy totalError or previous error that will screw up
                // the subsequent getOutput() calls where it needs the previous error states to compute the I and D terms
                // correctly.
                //
                if (xPosPidCtrl != null)
                {
                    xPosPidCtrl.setTarget(relativeTargetPose.x, resetError);
                }
                yPosPidCtrl.setTarget(relativeTargetPose.y, resetError);
            }

            if (targetHeadingOffset != null)
            {
                Double headingOffset = targetHeadingOffset.getOffset();
                if (headingOffset != null)
                {
                    turnPidCtrl.setTarget(driveBase.getHeading() + headingOffset, warpSpace, resetError);
                }
                else
                {
                    turnPidCtrl.setTarget(
                        relativeTargetPose.angle + referencePose.angle + robotPose.angle, warpSpace, resetError);
                }
            }
            else
            {
                turnPidCtrl.setTarget(
                    relativeTargetPose.angle + referencePose.angle + robotPose.angle, warpSpace, resetError);
            }
            velPidCtrl.setTarget(targetPoint.velocity, resetError);
            resetError = false;

            double xPosPower = xPosPidCtrl != null? xPosPidCtrl.getOutput(): 0.0;
            double yPosPower = yPosPidCtrl.getOutput();
            double turnPower = turnPidCtrl.getOutput();
            double velPower = targetPoint.velocity > 0.0? velPidCtrl.getOutput(): 0.0;
            double theta = Math.atan2(relativeTargetPose.x, relativeTargetPose.y);
            xPosPower = xPosPidCtrl == null? 0.0: TrcUtil.clipRange(xPosPower + velPower * Math.sin(theta),
                                                                    -moveOutputLimit, moveOutputLimit);
            yPosPower = TrcUtil.clipRange(yPosPower + velPower * Math.cos(theta), -moveOutputLimit, moveOutputLimit);
            turnPower = TrcUtil.clipRange(turnPower, -rotOutputLimit, rotOutputLimit);

            tracer.traceDebug(
                instanceName,
                "[" + pathIndex +
                "] RobotPose=" + robotPose +
                ",TargetPose=" + targetPoint.pose +
                ",relPose=" + relativeTargetPose);
            tracer.traceDebug(
                instanceName,
                "RobotVel=" + getVelocityInput() +
                ",TargetVel=" + targetPoint.velocity +
                ",xError=" + (xPosPidCtrl != null? xPosPidCtrl.getError(): 0.0) +
                ",yError=" + yPosPidCtrl.getError() +
                ",turnError=" + turnPidCtrl.getError() +
                ",velError=" + velPidCtrl.getError() +
                ",theta=" + Math.toDegrees(theta) +
                ",xPower=" + xPosPower +
                ",yPower=" + yPosPower +
                ",turnPower=" + turnPower +
                ",velPower=" + velPower);

            // If we have timed out or finished, stop the operation.
            double currTime = TrcTimer.getCurrentTime();
            boolean timedOut = currTime >= timedOutTime;

            stalled = (xPosPidCtrl == null || xPosPidCtrl.isStalled()) &&
                    yPosPidCtrl.isStalled() && turnPidCtrl.isStalled();
            boolean posOnTarget =
                (xPosPidCtrl == null || xPosPidCtrl.isOnTarget(posTolerance)) && yPosPidCtrl.isOnTarget(posTolerance);
            boolean headingOnTarget = turnPidCtrl.isOnTarget(turnTolerance);

            if (stalled || timedOut)
            {
                if (beepDevice != null)
                {
                    beepDevice.playTone(beepFrequency, beepDuration);
                }
            }

            if (timedOut || lastSegment && (stalled ||posOnTarget && headingOnTarget))
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
                    "\" Delta=\"" + relativeTargetPose + "\"");
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
        if (!TrcUtil.inRange(weight, 0.0, 1.0))
        {
            throw new IllegalArgumentException("Weight must be in range [0,1]!");
        }

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

        return (1.0 - weight) * startValue + weight * endValue;
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
        double turningRadius = proximityRadius + posTolerance;
        if (robotPose == null || robotPose.distanceTo(point2.pose) <= turningRadius)
        {
            if (robotPose != null)
            {
                //
                // For non-holonomic drivebase and the end-waypoint is within the robot's proximity circle +
                // posTolerance, we will interpolate the heading weight differently than holonomic drivebase.
                // The heading weight is the percentage distance of the robot position to the end-waypoint over
                // proximity radius.
                //
                weight = 1 - point2.pose.distanceTo(point1.pose)*(1 - weight)/turningRadius;
            }
            heading = interpolate(
                point1.pose.angle, warpSpace.getOptimizedTarget(point2.pose.angle, point1.pose.angle), weight);
        }
        else
        {
            //
            // For non-holonomic drivebase, maintain the robot heading pointing to the end-waypoint unless the
            // end-waypoint is within the robot's proximity circle.
            //
            TrcPose2D endpointPose = point2.pose.clone();
            endpointPose.angle = point1.pose.angle;
            heading = point1.pose.angle + endpointPose.relativeTo(robotPose).angle;
        }

        return new TrcWaypoint(timestep, new TrcPose2D(x, y, heading), position, velocity, acceleration, jerk);
    }   //interpolate

    /**
     * This method calculates the waypoint on the path segment that intersects the robot's proximity circle that is
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
        if (fastModeEnabled && robotPose.distanceTo(endWaypoint.getPositionPose()) > proximityRadius)
        {
            tracer.traceDebug(
                instanceName,
                "pathIndex=" + pathIndex +
                ", startPose=" + startWaypoint.getPositionPose() +
                ", endPose=" + endWaypoint.getPositionPose());
            return interpolate(
                startWaypoint, endWaypoint, 1.0, !incrementalTurn? robotPose: null);
        }
        else
        {
            // Find intersection of path segment with the proximity circle of the robot.
            RealVector startVector = startWaypoint.getPositionPose().toPosVector();
            RealVector endVector = endWaypoint.getPositionPose().toPosVector();
            RealVector robotVector = robotPose.toPosVector();

            RealVector startToEnd = endVector.subtract(startVector);
            RealVector robotToStart = startVector.subtract(robotVector);
            // Solve quadratic formula
            double a = startToEnd.dotProduct(startToEnd);
            double b = 2 * robotToStart.dotProduct(startToEnd);
            double c = robotToStart.dotProduct(robotToStart) - proximityRadius * proximityRadius;

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
                //
                // t1 and t2 represent the relative positions of the intersection points on the line segment. If they are
                // in the range of 0.0 and 1.0, they are on the line segment. Otherwise, the intersection points are
                // outside of the line segment. If the relative position is towards 0.0, it is closer to the start
                // waypoint of the line segment. If the relative position is towards 1.0, it is closer to the end
                // waypoint of the line segment.
                //
                // t represents the furthest intersection point (the one closest to the end waypoint of the line segment).
                //
                double t1 = (-b - discriminant)/(2*a);
                double t2 = (-b + discriminant)/(2*a);
                double t = Math.max(t1, t2);

                if (!TrcUtil.inRange(t, 0.0, 1.0))
                {
                    //
                    // The furthest intersection point is not on the line segment, so skip this segment.
                    //
                    tracer.traceDebug(
                        instanceName,
                        "Intersection not on line segment t1=" + t1 + ", t2=" + t2 + ", t=" + t +
                        ", stalled=" + stalled);
                    return null;
                }

                TrcWaypoint interpolated =
                    interpolate(startWaypoint, endWaypoint, t, xPosPidCtrl == null? robotPose: null);

                tracer.traceDebug(
                    instanceName,
                    "startPoint=" + startWaypoint.getPositionPose() +
                    ", endPoint=" + endWaypoint.getPositionPose() +
                    ", interpolatedPoint=" + interpolated.getPositionPose());

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
        //
        // Find the next segment that intersects with the proximity circle of the robot.
        // If there are tiny segments that are completely within the proximity circle, we will skip them all.
        //
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
                    //
                    // We are moving to the next waypoint.
                    //
                    if (waypointEventHandler != null && segmentStart.index != -1)
                    {
                        waypointEventHandler.waypointEvent(segmentStart.index, segmentStart);
                    }

                    tracer.traceDebug(
                        instanceName,
                        "Segment[" + (i - 1) +
                        ":" + segmentStart +
                        "->" + i +
                        ":" + segmentEnd +
                        "] PrevIndex=" + pathIndex +
                        ", Target=" + interpolated);
                    pathIndex = i;
                }
                return interpolated;
            }
            else if (stalled)
            {
                tracer.traceInfo(instanceName, "Segment " + i + " is stalled, moving to the next segment.");
                pathIndex = i;
            }
        }
        //
        // Found no intersection. The robot must be off-path. Just proceed to the immediate next waypoint.
        //
        return path.getWaypoint(pathIndex);
    }   //getFollowingPoint

}   //class TrcPurePursuitDrive