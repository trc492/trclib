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

package trclib.pathdrive;

import trclib.controller.TrcPidController;
import trclib.driverio.TrcTone;
import trclib.robotcore.TrcLoopProfiler;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.robotcore.TrcTaskMgr.TaskType;
import trclib.dataprocessor.TrcWarpSpace;
import trclib.drivebase.TrcDriveBase;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcRobotBattery;
import trclib.timer.TrcTimer;

/**
 * This class implements a PID controlled robot drive. A PID controlled robot drive consist of a robot drive base
 * and three PID controllers, one for the X direction, one for the Y direction and one for turn. If the robot drive
 * base is incapable of moving in the X direction, the X PID controller will be null. In addition, it has stall
 * detection support which will detect drive base stall condition. The drive base could stall if the robot runs into
 * an obstacle in low power or the robot is very close to target and doesn't have enough power to overcome steady
 * state error. When stall condition is detected, PID drive will be aborted so that the robot won't get stuck waiting
 * forever trying to reach target.
 */
public class TrcPidDrive
{
//    /**
//     * This interface provides a stuck wheel notification handler. It is useful for detecting drive base motor
//     * malfunctions. A stuck wheel could happen if the motor is malfunctioning, the motor power wire is unplugged,
//     * the motor encoder is malfunctioning or the motor encoder wire is unplugged.
//     */
//    public interface StuckWheelHandler
//    {
//        /**
//         * This method is called when a stuck wheel is detected.
//         *
//         * @param pidDrive specifies this TrcPidDrive instance.
//         * @param index specifies which wheel in the DriveBase is stuck.
//         */
//        void stuckWheel(TrcPidDrive pidDrive, int index);
//
//    }   //interface StuckWheelHandler

    /**
     * Turn mode specifies how PID controlled drive is turning the robot.
     */
    public enum TurnMode
    {
        IN_PLACE,
        PIVOT_FORWARD,
        PIVOT_BACKWARD,
        CURVE
    }   //enum TurnMode

    private static final double DEF_BEEP_FREQUENCY = 880.0; //in Hz
    private static final double DEF_BEEP_DURATION = 0.2;    //in seconds

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcDriveBase driveBase;
    private final TrcPidController xPidCtrl;
    private final TrcPidController yPidCtrl;
    private final TrcPidController turnPidCtrl;
    public final TrcLoopProfiler pidDriveTaskProfiler;
    private double xTolerance;
    private double yTolerance;
    private double turnTolerance;
    private final TrcTaskMgr.TaskObject driveTaskObj;
    private final TrcTaskMgr.TaskObject stopTaskObj;
    // Tracer config.
    private boolean logRobotPoseEvents = false;
    private boolean tracePidInfo = false;
    private boolean verbosePidInfo = false;
    private TrcRobotBattery battery = null;

    private boolean savedReferencePose = false;
    private TrcWarpSpace warpSpace = null;
    private boolean warpSpaceEnabled = true;
    private boolean absTargetModeEnabled = false;
    private boolean noOscillation = false;
//    private StuckWheelHandler stuckWheelHandler = null;
//    private double stuckTimeout = 0.0;
    private TurnMode turnMode = TurnMode.IN_PLACE;
    private TrcTone beepDevice = null;
    private double beepFrequency = DEF_BEEP_FREQUENCY;
    private double beepDuration = DEF_BEEP_DURATION;
    private TrcEvent notifyEvent = null;
    private double expiredTime = 0.0;
    private double manualX = 0.0;
    private double manualY = 0.0;
    private boolean active = false;
    private boolean holdTarget = false;
    private boolean turnOnly = false;
    private boolean maintainHeading = false;
    private boolean canceled = false;
    private String owner = null;
    private TrcPose2D absTargetPose;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the drive base object.
     * @param xPidCoeffs specifies the PID coefficients of the X PID controller.
     * @param xTolerance specifies the X PID tolerance.
     * @param xPidInput specifies the method to call to get X input.
     * @param yPidCoeffs specifies the PID coefficients of the Y PID controller.
     * @param yTolerance specifies the Y PID tolerance.
     * @param yPidInput specifies the method to call to get Y input.
     * @param turnPidCoeffs specifies the PID coefficients of the turn PID controller.
     * @param turnTolerance specifies the Turn PID tolerance.
     * @param turnPidInput specifies the method to call to get turn input.
     */
    public TrcPidDrive(
        String instanceName, TrcDriveBase driveBase,
        TrcPidController.PidCoefficients xPidCoeffs, double xTolerance, TrcPidController.PidInput xPidInput,
        TrcPidController.PidCoefficients yPidCoeffs, double yTolerance, TrcPidController.PidInput yPidInput,
        TrcPidController.PidCoefficients turnPidCoeffs, double turnTolerance, TrcPidController.PidInput turnPidInput)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.driveBase = driveBase;
        this.xPidCtrl = xPidCoeffs != null?
            new TrcPidController(instanceName + ".xPidCtrl", xPidCoeffs, xPidInput): null;
        this.yPidCtrl = new TrcPidController(instanceName + ".yPidCtrl", yPidCoeffs, yPidInput);
        this.turnPidCtrl = new TrcPidController(instanceName + "turnPidCtrl", turnPidCoeffs, turnPidInput);
        this.xTolerance = xTolerance;
        this.yTolerance = yTolerance;
        this.turnTolerance = turnTolerance;
        setAbsoluteTargetPose(driveBase.getFieldPosition());
        driveTaskObj = TrcTaskMgr.createTask(instanceName + ".driveTask", this::driveTask);
        stopTaskObj = TrcTaskMgr.createTask(instanceName + ".stopTask", this::stopTask);

        if (turnPidCtrl.hasAbsoluteSetPoint())
        {
            warpSpace = new TrcWarpSpace(instanceName, 0.0, 360.0);
        }

        pidDriveTaskProfiler = new TrcLoopProfiler(instanceName);
    }   //TrcPidDrive

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the drive base object.
     * @param yPidCoeffs specifies the PID coefficients of the Y PID controller.
     * @param yTolerance specifies the Y PID tolerance.
     * @param yPidInput specifies the method to call to get Y input.
     * @param turnPidCoeffs specifies the PID coefficients of the turn PID controller.
     * @param turnTolerance specifies the Turn PID tolerance.
     * @param turnPidInput specifies the method to call to get turn input.
     */
    public TrcPidDrive(
        String instanceName, TrcDriveBase driveBase,
        TrcPidController.PidCoefficients yPidCoeffs, double yTolerance, TrcPidController.PidInput yPidInput,
        TrcPidController.PidCoefficients turnPidCoeffs, double turnTolerance, TrcPidController.PidInput turnPidInput)
    {
        this(instanceName, driveBase, null, 0.0, null,
             yPidCoeffs, yTolerance, yPidInput, turnPidCoeffs, turnTolerance, turnPidInput);
    }   //TrcPidDrive

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
     * This method sets the PID tolerances for X, Y and Turn.
     *
     * @param xTolerance specifies X PID tolerance.
     * @param yTolerance specifies Y PID tolerance.
     * @param turnTolerance specifies Turn PID tolerance.
     */
    public synchronized void setPidTolerances(double xTolerance, double yTolerance, double turnTolerance)
    {
        this.xTolerance = xTolerance;
        this.yTolerance = yTolerance;
        this.turnTolerance = turnTolerance;
    }   //setPidTolerances

    /**
     * This method sets the PID tolerances for X, Y and Turn.
     *
     * @param yTolerance specifies Y PID tolerance.
     * @param turnTolerance specifies Turn PID tolerance.
     */
    public synchronized void setPidTolerances(double yTolerance, double turnTolerance)
    {
        setPidTolerances(0.0, yTolerance, turnTolerance);
    }   //setPidTolerances

    /**
     * This method enables/disables absolute target mode. This class always keep track of the absolute position of
     * the robot. When absolute target mode is enabled, all setRelativeTarget calls will adjust the relative targets
     * by subtracting the current robot position from the absolute target position. This will essentially canceling
     * out cumulative errors of multi-segment PID drive.
     *
     * @param enabled specifies true to enable absolute target mode, false to disable.
     */
    public synchronized void setAbsoluteTargetModeEnabled(boolean enabled)
    {
        this.absTargetModeEnabled = enabled;
        resetAbsoluteTargetPose();
    }   //setAbsoluteTargetModeEnabled

    /**
     * This method checks if Absolute Target Mode is enabled.
     *
     * @return true if Absolute Target Mode is enabled, false otherwise.
     */
    public synchronized boolean isAbsoluteTargetModeEnabled()
    {
        return absTargetModeEnabled;
    }   //isAbsoluteTargetModeEnabled

    /**
     * This method enables/disables NoOscillation mode. When NoOscillation is enabled, PIDDrive will determine which
     * degrees of freedom are moving and set the corresponding PID controllers to enable noOscillation mode. For
     * the degrees of freedom that are maintaining previous position, we need to allow oscillation.
     *
     * @param noOscillation specifies true to enable no oscillation, false to disable.
     */
    public synchronized void setNoOscillation(boolean noOscillation)
    {
        this.noOscillation = noOscillation;
    }   //setNoOscillation

    /**
     * This method enables/disables SQUID mode of the PID controllers.
     *
     * @param enable specifies true to enable and false to disable.
     */
    public synchronized void setSquidModeEnabled(boolean enable)
    {
        if (xPidCtrl != null)
        {
            xPidCtrl.setSquidModeEnabled(enable);
        }
        yPidCtrl.setSquidModeEnabled(enable);
        turnPidCtrl.setSquidModeEnabled(enable);
    }   //setSquidModeEnabled

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
     * This method checks if warpspace processing is enabled in a PID controlled turn.
     *
     * @return true if warpspace processing is enabled, false otherwise.
     */
    public boolean isWarpSpaceEnabled()
    {
        return warpSpaceEnabled;
    }   //isWarpSpaceEnabled

    /**
     * This method enables/disables warpspace processing when doing a PID controlled turn.
     *
     * @param enabled specifies true to enable warpspace processing, false to disable.
     */
    public void setWarpSpaceEnabled(boolean enabled)
    {
        warpSpaceEnabled = enabled;
    }   //setWarpSpaceEnabled

    /**
     * This method returns the current absolute target pose. It is useful if you have multiple instances of the
     * TrcEnhancedPidDrive that you want to sync their target poses.
     *
     * @return current absolute target pose.
     */
    public synchronized TrcPose2D getAbsoluteTargetPose()
    {
        return absTargetPose.clone();
    }   //getAbsoluteTargetPose

    /**
     * This method sets the current absolute target pose to the given pose.
     *
     * @param pose specifies the pose to be set as the current absolute target pose.
     */
    public synchronized void setAbsoluteTargetPose(TrcPose2D pose)
    {
        absTargetPose = pose;
    }   //setAbsoluteTargetPose

    /**
     * This method sets the current robot pose as the absolute target pose.
     */
    public synchronized void resetAbsoluteTargetPose()
    {
        if (absTargetModeEnabled)
        {
            setAbsoluteTargetPose(driveBase.getFieldPosition());
        }
    }   //resetAbsoluteTargetPose

    /**
     * This method sets the robot's current absolute pose to the given pose. It also updates the absolute target pose
     * to the same pose. This can be used to set the robot's absolute starting position relative to the origin of the
     * coordinate system.
     *
     * @param pose specifies the absolute pose of the robot relative to the origin of the coordinate system.
     */
    public synchronized void setAbsolutePose(TrcPose2D pose)
    {
        driveBase.setFieldPosition(pose);
        setAbsoluteTargetPose(pose);
    }   //setAbsolutePose

    /**
     * This method returns the robot's current absolute pose relative to the origin of the coordinate system.
     *
     * @return robot's current absolute pose.
     */
    public synchronized TrcPose2D getAbsolutePose()
    {
        return driveBase.getFieldPosition();
    }   //getAbsolutePose

    /**
     * This method returns the X PID controller if any.
     *
     * @return X PID controller.
     */
    public TrcPidController getXPidCtrl()
    {
        return xPidCtrl;
    }   //getXPidCtrl

    /**
     * This method returns the Y PID controller if any.
     *
     * @return Y PID controller.
     */
    public TrcPidController getYPidCtrl()
    {
        return yPidCtrl;
    }   //getYPidCtrl

    /**
     * This method returns the Turn PID controller if any.
     *
     * @return Turn PID controller.
     */
    public TrcPidController getTurnPidCtrl()
    {
        return turnPidCtrl;
    }   //getTurnPidCtrl

//    /**
//     * This method sets a stuck wheel handler to enable stuck wheel detection.
//     *
//     * @param stuckWheelHandler specifies the stuck wheel handler.
//     * @param stuckTimeout specifies the stuck timeout in seconds.
//     */
//    public synchronized void setStuckWheelHandler(StuckWheelHandler stuckWheelHandler, double stuckTimeout)
//    {
//        this.stuckWheelHandler = stuckWheelHandler;
//        this.stuckTimeout = stuckTimeout;
//    }   //setStuckWheelHandler

    /**
     * This methods sets the turn mode. Supported modes are in-place (default), pivot and curve.
     *
     * @param turnMode specifies the turn mode to set to.
     */
    public synchronized void setTurnMode(TurnMode turnMode)
    {
        this.turnMode = turnMode;
    }   //setTurnMode

    /**
     * This method returns the current turn mode.
     *
     * @return current turn mode.
     */
    public synchronized TurnMode getTurnMode()
    {
        return turnMode;
    }   //getTurnMode

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
        if (xPidCtrl != null)
        {
            xPidCtrl.setStallDetectionEnabled(stallDetectionDelay, stallDetectionTimeout, stallErrorRateThreshold);
        }
        yPidCtrl.setStallDetectionEnabled(stallDetectionDelay, stallDetectionTimeout, stallErrorRateThreshold);
        turnPidCtrl.setStallDetectionEnabled(stallDetectionDelay, stallDetectionTimeout, stallErrorRateThreshold);
    }   //setStallDetectionEnabled

    /**
     * This method enables/disables stall detection.
     *
     * @param enabled specifies true to enable stall detection, false to disable.
     */
    public void setStallDetectionEnabled(boolean enabled)
    {
        if (xPidCtrl != null)
        {
            xPidCtrl.setStallDetectionEnabled(enabled);
        }
        yPidCtrl.setStallDetectionEnabled(enabled);
        turnPidCtrl.setStallDetectionEnabled(enabled);
    }   //setStallDetectionEnabled

    /**
     * This method starts a PID operation by setting the PID targets.
     *
     * @param xTarget specifies the X target position.
     * @param yTarget specifies the Y target position.
     * @param turnTarget specifies the target angle.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    private void setTarget(
        double xTarget, double yTarget, double turnTarget, boolean holdTarget, TrcEvent event, double timeout)
    {
        double xError = 0.0, yError = 0.0, turnError = 0.0;

        tracer.traceDebug(
            instanceName,
            "x=" + xTarget +
            ", y=" + yTarget +
            ", turn=" + turnTarget +
            ", hold=" + holdTarget +
            ", event=" + event +
            ", timeout=" + timeout);
        if (xPidCtrl != null && yPidCtrl != null &&
            xPidCtrl.hasAbsoluteSetPoint() != yPidCtrl.hasAbsoluteSetPoint())
        {
            throw new IllegalStateException("X and Y PID controller must have the same absolute setpoint state.");
        }

        if (xPidCtrl != null && !xPidCtrl.hasAbsoluteSetPoint() || yPidCtrl != null && !yPidCtrl.hasAbsoluteSetPoint())
        {
            driveBase.pushReferenceOdometry();
            savedReferencePose = true;
        }

        if (xPidCtrl != null)
        {
            xPidCtrl.setTarget(xTarget);
            xPidCtrl.startStallDetection();
            xError = xPidCtrl.getError();
        }

        if (yPidCtrl != null)
        {
            yPidCtrl.setTarget(yTarget);
            yPidCtrl.startStallDetection();
            yError = yPidCtrl.getError();
        }

        if (turnPidCtrl != null)
        {
            turnPidCtrl.setTarget(turnTarget, warpSpaceEnabled? warpSpace: null);
            turnPidCtrl.startStallDetection();
            turnError = turnPidCtrl.getError();
        }

        if (event != null)
        {
            event.clear();
        }
        this.notifyEvent = event;

        double currTime = TrcTimer.getCurrentTime();
        expiredTime = timeout == 0.0 ? Double.POSITIVE_INFINITY : currTime + timeout;

        this.holdTarget = holdTarget;
        this.turnOnly = xError == 0.0 && yError == 0.0 && turnError != 0.0;

        setTaskEnabled(true);
    }   //setTarget

    /**
     * This method sets the PID controlled drive referencing sensors as targets. It is sometimes useful to use sensors
     * as target referencing devices. For example, this can be used to do vision target drive where the camera can
     * provide the target distance as well as the target angle. Note that when doing sensor target drive, you cannot
     * turn on Absolute Target Mode because Absolute Target Mode assumes wheel odometry is the target referencing
     * device.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     * @param xTarget specifies the X target position.
     * @param yTarget specifies the Y target position.
     * @param turnTarget specifies the target angle.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public synchronized void setSensorTarget(
        String owner, double xTarget, double yTarget, double turnTarget, boolean holdTarget, TrcEvent event,
        double timeout)
    {
        if (absTargetModeEnabled)
        {
            throw new UnsupportedOperationException("SensorTarget cannot use Absolute Target Mode.");
        }
        //
        // In setSensorTarget, we do not honor noOscillation because we generally want the sensor to achieve target
        // accurately in the degree of freedom that the sensor is monitoring. Since we don't know which degree of
        // freedom the caller want to maintain accuracy, we want to let the caller decide which PID controller to set
        // noOscillation mode ON.
        //
        if (noOscillation)
        {
            throw new UnsupportedOperationException("SensorTarget cannot have noOscaillation mode ON.");
        }

        if (driveBase.validateOwnership(owner))
        {
            this.owner = owner;
            setTarget(xTarget, yTarget, turnTarget, holdTarget, event, timeout);
        }
    }   //setSensorTarget

    /**
     * This method sets the PID controlled drive referencing sensors as targets. It is sometimes useful to use sensors
     * as target referencing devices. For example, this can be used to do vision target drive where the camera can
     * provide the target distance as well as the target angle. Note that when doing sensor target drive, you cannot
     * turn on Absolute Target Mode because Absolute Target Mode assumes wheel odometry is the target referencing
     * device.
     *
     * @param xTarget specifies the X target position.
     * @param yTarget specifies the Y target position.
     * @param turnTarget specifies the target angle.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setSensorTarget(
        double xTarget, double yTarget, double turnTarget, boolean holdTarget, TrcEvent event, double timeout)
    {
        setSensorTarget(null, xTarget, yTarget, turnTarget, holdTarget, event, timeout);
    }   //setSensorTarget

    /**
     * This method sets the PID controlled drive referencing sensors as targets. It is sometimes useful to use sensors
     * as target referencing devices. For example, this can be used to do vision target drive where the camera can
     * provide the target distance as well as the target angle. Note that when doing sensor target drive, you cannot
     * turn on Absolute Target Mode because Absolute Target Mode assumes wheel odometry is the target referencing
     * device.
     *
     * @param xTarget specifies the X target position.
     * @param yTarget specifies the Y target position.
     * @param turnTarget specifies the target angle.
     * @param event specifies an event object to signal when done.
     */
    public void setSensorTarget(double xTarget, double yTarget, double turnTarget, TrcEvent event)
    {
        setSensorTarget(null, xTarget, yTarget, turnTarget, false, event, 0.0);
    }   //setSensorTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     * @param xDelta specifies the X target relative to the current X position.
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param turnDelta specifies the turn target relative to the current angle.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public synchronized void setRelativeTarget(
        String owner, double xDelta, double yDelta, double turnDelta,
        boolean holdTarget, TrcEvent event, double timeout)
    {
        if (driveBase.validateOwnership(owner))
        {
            this.owner = owner;
            // adding relative X, Y and turn targets to the absolute target pose.
            TrcPose2D newAbsTargetPose = absTargetPose.addRelativePose(new TrcPose2D(xDelta, yDelta, turnDelta));
            double xTarget, yTarget, turnTarget;

            tracer.traceDebug(
                instanceName,
                "owner=" + owner +
                ", xDelta=" + xDelta +
                ", yDelta=" + yDelta +
                ", turnDelta=" + turnDelta +
                ", absTargetPose=" + absTargetPose +
                ", newAbsTargetPose=" + newAbsTargetPose);
            if (absTargetModeEnabled)
            {
                if (xDelta == 0.0 && yDelta == 0.0 && turnDelta != 0.0)
                {
                    //
                    // setTarget disables X and Y PID controllers while doing a turn-only movement but absolute target
                    // mode may calculate non-zero X and Y to compensate for previous errors. This hides the fact that
                    // it's a turn-only movement and causes the robot to do a combo movement that causes more odometry
                    // error. So we need to preserve the X and Y odometry before the turn and restore them afterwards.
                    //
                    xTarget = yTarget = 0.0;
                }
                else
                {
                    //
                    // Adjusting relative target by subtracting current robot pose from the absolute target pose.
                    // This will eliminate cumulative error.
                    //
                    TrcPose2D relativePose = newAbsTargetPose.relativeTo(driveBase.getFieldPosition());
                    xTarget = relativePose.x;
                    yTarget = relativePose.y;
                }
            }
            else
            {
                //
                // Not using absolute target pose, so use the relative targets as-is.
                //
                xTarget = xDelta;
                yTarget = yDelta;
            }
            //
            // Adjust the turn target.
            //
            turnTarget = turnPidCtrl.hasAbsoluteSetPoint()? newAbsTargetPose.angle : turnDelta;

            if (noOscillation)
            {
                //
                // In noOscillation mode, we want no oscillation only in the degrees of freedom that are actually
                // moving. For degrees of freedom that we are maintaining previous position, we need to allow
                // oscillation.
                //
                if (xPidCtrl != null)
                {
                    xPidCtrl.setNoOscillation(xDelta != 0.0);
                }

                if (yPidCtrl != null)
                {
                    yPidCtrl.setNoOscillation(yDelta != 0.0);
                }

                turnPidCtrl.setNoOscillation(turnDelta != 0.0);
            }

            tracer.traceDebug(
                instanceName,
                "xTarget=" + xTarget +
                " yTarget=" + yTarget +
                ", turnTarget=" + turnTarget +
                ", newAbsTargetPose=" + newAbsTargetPose);
            // The new target pose will become the updated absolute target pose.
            absTargetPose = newAbsTargetPose;
            setTarget(xTarget, yTarget, turnTarget, holdTarget, event, timeout);
        }
    }   //setRelativeTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param turnDelta specifies the turn target relative to the current angle.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setRelativeTarget(
        double xDelta, double yDelta, double turnDelta, boolean holdTarget, TrcEvent event, double timeout)
    {
        setRelativeTarget(null, xDelta, yDelta, turnDelta, holdTarget, event, timeout);
    }   //setRelativeTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param turnDelta specifies the turn target relative to the current angle.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setRelativeTarget(double xDelta, double yDelta, double turnDelta, TrcEvent event, double timeout)
    {
        setRelativeTarget(null, xDelta, yDelta, turnDelta, false, event, timeout);
    }   //setRelativeTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param turnDelta specifies the turn target relative to the current angle.
     * @param event specifies an event object to signal when done.
     */
    public void setRelativeTarget(double xDelta, double yDelta, double turnDelta, TrcEvent event)
    {
        setRelativeTarget(null, xDelta, yDelta, turnDelta, false, event, 0.0);
    }   //setRelativeTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param turnDelta specifies the turn target relative to the current angle.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     */
    public void setRelativeTarget(double xDelta, double yDelta, double turnDelta, boolean holdTarget)
    {
        setRelativeTarget(null, xDelta, yDelta, turnDelta, holdTarget, null, 0.0);
    }   //setRelativeTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setRelativeXYTarget(double xDelta, double yDelta, TrcEvent event, double timeout)
    {
        setRelativeTarget(null, xDelta, yDelta, 0.0, false, event, timeout);
    }   //setRelativeXYTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param event specifies an event object to signal when done.
     */
    public void setRelativeXYTarget(double xDelta, double yDelta, TrcEvent event)
    {
        setRelativeTarget(null, xDelta, yDelta, 0.0, false, event, 0.0);
    }   //setRelativeXYTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setRelativeXTarget(double xDelta, TrcEvent event, double timeout)
    {
        setRelativeTarget(null, xDelta, 0.0, 0.0, false, event, timeout);
    }   //setRelativeXTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param event specifies an event object to signal when done.
     */
    public void setRelativeXTarget(double xDelta, TrcEvent event)
    {
        setRelativeTarget(null, xDelta, 0.0, 0.0, false, event, 0.0);
    }   //setRelativeXTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setRelativeYTarget(double yDelta, TrcEvent event, double timeout)
    {
        setRelativeTarget(null, 0.0, yDelta, 0.0, false, event, timeout);
    }   //setRelativeYTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param event specifies an event object to signal when done.
     */
    public void setRelativeYTarget(double yDelta, TrcEvent event)
    {
        setRelativeTarget(null, 0.0, yDelta, 0.0, false, event, 0.0);
    }   //setRelativeYTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param turnDelta specifies the turn target relative to the current angle.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setRelativeTurnTarget(double turnDelta, TrcEvent event, double timeout)
    {
        setRelativeTarget(null, 0.0, 0.0, turnDelta, false, event, timeout);
    }   //setRelativeTurnTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param turnDelta specifies the turn target relative to the current angle.
     * @param event specifies an event object to signal when done.
     */
    public void setRelativeTurnTarget(double turnDelta, TrcEvent event)
    {
        setRelativeTarget(null, 0.0, 0.0, turnDelta, false, event, 0.0);
    }   //setRelativeTurnTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     * @param absX specifies the absolute X target position.
     * @param absY specifies the absolute Y target position.
     * @param absHeading specifies the absolute target angle.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public synchronized void setAbsoluteTarget(
        String owner, double absX, double absY, double absHeading, boolean holdTarget, TrcEvent event, double timeout)
    {
        if (driveBase.validateOwnership(owner))
        {
            this.owner = owner;

            TrcPose2D newTargetPose = new TrcPose2D(absX, absY, absHeading);
            TrcPose2D currRobotPose = driveBase.getFieldPosition();
            //
            // RelativeTo will calculate delta X and delta Y between the target and current positions relative to the
            // field coordinates and it will rotate the (deltaX, deltaY) vector to the robot's heading, essentially
            // returning deltaX and deltaY relative to the robot's heading instead of the field coordinates.
            // Unfortunately, the current heading of the robot may not be the intended heading due to turn error.
            // Therefore, before passing currRobotPose to relativeTo, we need to adjust the angle to the intended
            // heading or else we will incorporate the heading error into the deltaX and deltaY calculation.
            //
            currRobotPose.angle = absHeading;
            TrcPose2D relativePose = newTargetPose.relativeTo(currRobotPose);
            double turnTarget = turnPidCtrl.hasAbsoluteSetPoint()?
                    newTargetPose.angle : newTargetPose.angle - currRobotPose.angle;

            tracer.traceDebug(
                instanceName,
                "owner=" + owner +
                ", absX=" + absX +
                ", absY=" + absY +
                ", absHeading=" + absHeading +
                ", currPose=" + currRobotPose +
                ", absTargetPose=" + absTargetPose);
            tracer.traceDebug(
                instanceName,
                "xTarget=" + relativePose.x +
                ", yTarget=" + relativePose.y +
                ", turnTarget=" + turnTarget +
                ", newPose=" + newTargetPose);
            if (noOscillation)
            {
                //
                // In noOscillation mode, we want no oscillation only in the degrees of freedom that are actually
                // moving. For degrees of freedom that we are maintaining previous position, we need to allow
                // oscillation.
                //
                if (xPidCtrl != null)
                {
                    xPidCtrl.setNoOscillation(absX != absTargetPose.x);
                }

                if (yPidCtrl != null)
                {
                    yPidCtrl.setNoOscillation(absY != absTargetPose.y);
                }

                turnPidCtrl.setNoOscillation(absHeading != absTargetPose.angle);
            }

            absTargetPose = newTargetPose;
            setTarget(relativePose.x, relativePose.y, turnTarget, holdTarget, event, timeout);
        }
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absX specifies the absolute X target position.
     * @param absY specifies the absolute Y target position.
     * @param absHeading specifies the absolute target angle.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setAbsoluteTarget(
        double absX, double absY, double absHeading, boolean holdTarget, TrcEvent event, double timeout)
    {
        setAbsoluteTarget(null, absX, absY, absHeading, holdTarget, event, timeout);
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absX specifies the absolute X target position.
     * @param absY specifies the absolute Y target position.
     * @param absHeading specifies the absolute target angle.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setAbsoluteTarget(double absX, double absY, double absHeading, TrcEvent event, double timeout)
    {
        setAbsoluteTarget(null, absX, absY, absHeading, false, event, timeout);
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absX specifies the absolute X target position.
     * @param absY specifies the absolute Y target position.
     * @param absHeading specifies the absolute target angle.
     * @param event specifies an event object to signal when done.
     */
    public void setAbsoluteTarget(double absX, double absY, double absHeading, TrcEvent event)
    {
        setAbsoluteTarget(null, absX, absY, absHeading, false, event, 0.0);
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absX specifies the absolute X target position.
     * @param absY specifies the absolute Y target position.
     * @param absHeading specifies the absolute target angle.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     */
    public void setAbsoluteTarget(double absX, double absY, double absHeading, boolean holdTarget)
    {
        setAbsoluteTarget(null, absX, absY, absHeading, holdTarget, null, 0.0);
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     * @param targetPose specifies the target position.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setAbsoluteTarget(
        String owner, TrcPose2D targetPose, boolean holdTarget, TrcEvent event, double timeout)
    {
        setAbsoluteTarget(owner, targetPose.x, targetPose.y, targetPose.angle, holdTarget, event, timeout);
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param targetPose specifies the target position.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setAbsoluteTarget(TrcPose2D targetPose, boolean holdTarget, TrcEvent event, double timeout)
    {
        setAbsoluteTarget(null, targetPose.x, targetPose.y, targetPose.angle, holdTarget, event, timeout);
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param targetPose specifies the target position.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setAbsoluteTarget(TrcPose2D targetPose, TrcEvent event, double timeout)
    {
        setAbsoluteTarget(null, targetPose.x, targetPose.y, targetPose.angle, false, event, timeout);
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param targetPose specifies the target position.
     * @param event specifies an event object to signal when done.
     */
    public void setAbsoluteTarget(TrcPose2D targetPose, TrcEvent event)
    {
        setAbsoluteTarget(null, targetPose.x, targetPose.y, targetPose.angle, false, event, 0.0);
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param targetPose specifies the target position.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     */
    public void setAbsoluteTarget(TrcPose2D targetPose, boolean holdTarget, TrcEvent event)
    {
        setAbsoluteTarget(null, targetPose.x, targetPose.y, targetPose.angle, holdTarget, event, 0.0);
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param targetPose specifies the target position.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     */
    public void setAbsoluteTarget(TrcPose2D targetPose, boolean holdTarget)
    {
        setAbsoluteTarget(null, targetPose.x, targetPose.y, targetPose.angle, holdTarget, null, 0.0);
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absX specifies the absolute X target position.
     * @param absY specifies the absolute Y target position.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setAbsoluteXYTarget(double absX, double absY, TrcEvent event, double timeout)
    {
        setAbsoluteTarget(null, absX, absY, absTargetPose.angle, false, event, timeout);
    }   //setAbsoluteXYTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absX specifies the absolute X target position.
     * @param absY specifies the absolute Y target position.
     * @param event specifies an event object to signal when done.
     */
    public void setAbsoluteXYTarget(double absX, double absY, TrcEvent event)
    {
        setAbsoluteTarget(null, absX, absY, absTargetPose.angle, false, event, 0.0);
    }   //setAbsoluteXYTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absX specifies the absolute X target position.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setAbsoluteXTarget(double absX, TrcEvent event, double timeout)
    {
        setAbsoluteTarget(null, absX, absTargetPose.y, absTargetPose.angle, false, event, timeout);
    }   //setAbsoluteXTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absX specifies the absolute X target position.
     * @param event specifies an event object to signal when done.
     */
    public void setAbsoluteXTarget(double absX, TrcEvent event)
    {
        setAbsoluteTarget(null, absX, absTargetPose.y, absTargetPose.angle, false, event, 0.0);
    }   //setAbsoluteXTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absY specifies the absolute Y target position.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setAbsoluteYTarget(double absY, TrcEvent event, double timeout)
    {
        setAbsoluteTarget(null, absTargetPose.x, absY, absTargetPose.angle, false, event, timeout);
    }   //setAbsoluteYTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absY specifies the absolute Y target position.
     * @param event specifies an event object to signal when done.
     */
    public void setAbsoluteYTarget(double absY, TrcEvent event)
    {
        setAbsoluteTarget(null, absTargetPose.x, absY, absTargetPose.angle, false, event, 0.0);
    }   //setAbsoluteYTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absHeading specifies the absolute target angle.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setAbsoluteHeadingTarget(double absHeading, TrcEvent event, double timeout)
    {
        //
        // Use the current absolute pose for X and Y to ensure that
        // we do not find any error along X or Y, thereby ensuring that the X
        // and Y PID controllers are turned off during the turn.
        //
        final TrcPose2D currentAbsPose = driveBase.getFieldPosition();
        setAbsoluteTarget(null, currentAbsPose.x, currentAbsPose.y, absHeading, false, event, timeout);
    }   //setAbsoluteHeadingTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absHeading specifies the absolute target angle.
     * @param event specifies an event object to signal when done.
     */
    public void setAbsoluteHeadingTarget(double absHeading, TrcEvent event)
    {
        setAbsoluteHeadingTarget(absHeading, event, 0.0);
    }   //setAbsoluteHeadingTarget

    /**
     * This method allows a mecanum drive base to drive and maintain a fixed angle.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     * @param xPower specifies the X drive power.
     * @param yPower specifies the Y drive power.
     * @param headingTarget specifies the angle to maintain.
     */
    public synchronized void driveMaintainHeading(String owner, double xPower, double yPower, double headingTarget)
    {
        if (driveBase.validateOwnership(owner))
        {
            this.owner = owner;

            if (xPidCtrl != null)
            {
                manualX = xPower;
                manualY = yPower;
                if (turnPidCtrl != null)
                {
                    turnPidCtrl.setTarget(headingTarget);
                    turnPidCtrl.startStallDetection();
                }
                maintainHeading = true;
                setTaskEnabled(true);
            }
        }
    }   //driveMaintainHeading

    /**
     * This method allows a mecanum drive base to drive and maintain a fixed angle.
     *
     * @param xPower specifies the X drive power.
     * @param yPower specifies the Y drive power.
     * @param headingTarget specifies the angle to maintain.
     */
    public synchronized void driveMaintainHeading(double xPower, double yPower, double headingTarget)
    {
        driveMaintainHeading(null, xPower, yPower, headingTarget);
    }   //driveMaintainHeading

    /**
     * This method checks if a PID drive operation is currently active.
     *
     * @return true if PID drive is active, false otherwise.
     */
    public boolean isActive()
    {
        return active;
    }   //isActive

    /**
     * This method cancels an active PID drive operation and stops all motors.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     */
    public synchronized void cancel(String owner)
    {
        tracer.traceDebug(instanceName, "Canceling PID drive.");
        if (active && driveBase.validateOwnership(owner))
        {
            stopPid(owner);
            canceled = true;
            if (notifyEvent != null)
            {
                notifyEvent.cancel();
                notifyEvent = null;
            }
        }
    }   //cancel

    /**
     * This method cancels an active PID drive operation and stops all motors.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method checks if a PID drive operation was canceled.
     *
     * @return true if PID drive is active, false otherwise.
     */
    public boolean isCanceled()
    {
        return canceled;
    }   //isCanceled

    /**
     * This method stops the PID drive operation and reset the states.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     */
    private synchronized void stopPid(String owner)
    {
        tracer.traceDebug(instanceName, "Stopping PID.");
        setTaskEnabled(false);
        driveBase.stop(owner);

        if (xPidCtrl != null)
        {
            xPidCtrl.reset();
        }

        if (yPidCtrl != null)
        {
            yPidCtrl.reset();
        }

        if (turnPidCtrl != null)
        {
            turnPidCtrl.reset();
        }

        // restore the old reference pose.
        if (savedReferencePose)
        {
            driveBase.popReferenceOdometry();
            savedReferencePose = false;
        }

        holdTarget = false;
        turnOnly = false;
        maintainHeading = false;
        canceled = false;
        owner = null;
    }   //stopPid

    /**
     * This method enables/disables the PID drive task. It assumes the caller has the synchronized lock.
     *
     * @param enabled specifies true to enable PID drive task, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            driveTaskObj.registerTask(TaskType.POST_PERIODIC_TASK);
            stopTaskObj.registerTask(TaskType.STOP_TASK);
        }
        else
        {
            driveTaskObj.unregisterTask();
            stopTaskObj.unregisterTask();
        }
        active = enabled;
    }   //setTaskEnabled

    /**
     * This method is called periodically to execute the PID drive operation.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private synchronized void driveTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        pidDriveTaskProfiler.recordLoopStartTime();
        long startNanoTime = TrcTimer.getNanoTime();
        double xPower = turnOnly || xPidCtrl == null? 0.0: xPidCtrl.calculate();
        double yPower = turnOnly || yPidCtrl == null? 0.0: yPidCtrl.calculate();
        double turnPower = turnPidCtrl == null? 0.0: turnPidCtrl.calculate();

        double currTime = TrcTimer.getCurrentTime();
        boolean expired = currTime >= expiredTime;
        boolean xOnTarget = xPidCtrl == null || xPidCtrl.isOnTarget(xTolerance);
        boolean yOnTarget = yPidCtrl == null || yPidCtrl.isOnTarget(yTolerance);
        boolean turnOnTarget = turnPidCtrl == null || turnPidCtrl.isOnTarget(turnTolerance);
        boolean onTarget = turnOnTarget && (turnOnly || xOnTarget && yOnTarget);
        boolean stalled = (xPidCtrl == null || xPidCtrl.isStalled()) &&
                          (yPidCtrl == null || yPidCtrl.isStalled()) &&
                          (turnPidCtrl == null || turnPidCtrl.isStalled());
        pidDriveTaskProfiler.recordProfilePointElapsedTime("UpdateStates", startNanoTime, true);

//        if (stuckWheelHandler != null)
//        {
//            for (int i = 0; i < driveBase.getNumMotors(); i++)
//            {
//                if(driveBase.isMotorStalled(i, stuckTimeout))
//                {
//                    stuckWheelHandler.stuckWheel(this, i);
//                }
//            }
//        }

        if (stalled || expired)
        {
            if (beepDevice != null)
            {
                beepDevice.playTone(beepFrequency, beepDuration);
            }
            tracer.traceInfo(instanceName, "Stalled=" + stalled + ", Expired=" + expired);
        }

        if (maintainHeading && driveBase.supportsHolonomicDrive())
        {
            startNanoTime = TrcTimer.getNanoTime();
            driveBase.holonomicDrive(owner, manualX, manualY, turnPower, false, 0.0);
            pidDriveTaskProfiler.recordProfilePointElapsedTime("MaintainningHeading", startNanoTime, true);
        }
        else if (expired || stalled || onTarget)
        {
            startNanoTime = TrcTimer.getNanoTime();
            if (expired || stalled || !holdTarget)
            {
                stopPid(owner);
            }
            // If we come here, both onTarget and holdTarget are true.
            // We will stop the drive base but not stopping PID.
            else
            {
                driveBase.stop(owner);
            }
            // If we are stalled, timed out or on target, we will signal the event regardless whether we are holding
            // target or not. This means that if we are holding target, the caller is responsible for canceling PID
            // eventually.
            if (notifyEvent != null)
            {
                notifyEvent.signal();
                notifyEvent = null;
            }

            if (xPidCtrl != null) xPidCtrl.endStallDetection();
            if (yPidCtrl != null) yPidCtrl.endStallDetection();
            if (turnPidCtrl != null) turnPidCtrl.endStallDetection();
            pidDriveTaskProfiler.recordProfilePointElapsedTime("Finish", startNanoTime, true);
        }
        // If we come here, we are not on target yet, keep driving.
        else if (xPidCtrl != null && driveBase.supportsHolonomicDrive())
        {
            startNanoTime = TrcTimer.getNanoTime();
            driveBase.holonomicDrive(owner, xPower, yPower, turnPower);
            pidDriveTaskProfiler.recordProfilePointElapsedTime("Drive", startNanoTime, true);
        }
        else if (turnOnly)
        {
            startNanoTime = TrcTimer.getNanoTime();
            switch (turnMode)
            {
                case IN_PLACE:
                    driveBase.arcadeDrive(owner, 0.0, turnPower, false);
                    break;

                case PIVOT_FORWARD:
                case CURVE:
                    if (turnPower < 0.0)
                    {
                        driveBase.tankDrive(owner, 0.0, -turnPower);
                    }
                    else
                    {
                        driveBase.tankDrive(owner, turnPower, 0.0);
                    }
                    break;

                case PIVOT_BACKWARD:
                    if (turnPower < 0.0)
                    {
                        driveBase.tankDrive(owner, turnPower, 0.0);
                    }
                    else
                    {
                        driveBase.tankDrive(owner, 0.0, -turnPower);
                    }
                    break;
            }
            pidDriveTaskProfiler.recordProfilePointElapsedTime("TurnOnly", startNanoTime, true);
        }
        else if (turnMode == TurnMode.IN_PLACE)
        {
            // We are still in an in-place turn.
            startNanoTime = TrcTimer.getNanoTime();
            driveBase.arcadeDrive(owner, yPower, turnPower, false);
            pidDriveTaskProfiler.recordProfilePointElapsedTime("TurnInPlace", startNanoTime, true);
        }
        else
        {
            startNanoTime = TrcTimer.getNanoTime();
            driveBase.curveDrive(owner, yPower, turnPower, false);
            pidDriveTaskProfiler.recordProfilePointElapsedTime("CurveTurn", startNanoTime, true);
        }

        startNanoTime = TrcTimer.getNanoTime();
        if (logRobotPoseEvents)
        {
            tracer.logEvent(
                instanceName, "RobotPose", "pose=\"" + driveBase.getFieldPosition() + "\"");
        }
        pidDriveTaskProfiler.recordProfilePointElapsedTime("LogPoseEvent", startNanoTime, true);

        startNanoTime = TrcTimer.getNanoTime();
        if (tracePidInfo)
        {
            if (xPidCtrl != null) xPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
            if (yPidCtrl != null) yPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
            if (turnPidCtrl != null) turnPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
        }
        pidDriveTaskProfiler.recordProfilePointElapsedTime("TracePidIinfo", startNanoTime, true);
    }   //driveTask

    /**
     * This method is called when the competition mode is about to end to stop the PID drive operation if any.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void stopTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        stopPid(owner);
    }   //stopTask

}   //class TrcPidDrive
