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

package trclib.command;

import java.util.Arrays;

import trclib.output.TrcDashboard;
import trclib.robotcore.TrcDbgTrace;
import trclib.drivebase.TrcDriveBase;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcPidController;
import trclib.robotcore.TrcPidController.PidCoefficients;
import trclib.pathdrive.TrcPidDrive;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements a generic PID control drive command. It is agnostic to the PID controller sensors.
 * The caller provides the PID drive object that has all PID controllers which means the caller controls
 * what sensors are controlling the X, Y and turn PID controllers. For example, the caller can provide a PID
 * drive object that uses the encoders to control the X and Y PID controllers and a gyro for the turn PID
 * controller. The caller can also use the encoders to control the X and Y PID controllers but a camera to
 * control the turn PID controller.
 */
public class CmdPidDrive implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        PREP_FOR_TUNING,
        PID_DRIVE,
        DONE
    }   //enum State

    private static final String moduleName = CmdPidDrive.class.getSimpleName();

    private final TrcDashboard dashboard = TrcDashboard.getInstance();
    private final TrcDbgTrace tracer = new TrcDbgTrace();
    private final TrcDriveBase driveBase;
    private final TrcPidDrive pidDrive;
    private final double delay;
    private final boolean useSensorOdometry;
    private final TrcPidController.PidCoefficients tunePidCoeff;
    private final TrcPose2D[] pathPoints;
    private int pathIndex;

    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final TrcPidController xPidCtrl;
    private final TrcPidController yPidCtrl;
    private final TrcPidController turnPidCtrl;

    private TrcPidController tunePidCtrl = null;
    private PidCoefficients savedPidCoeff = null;
    private Boolean savedTargetIsAbsolute = null;
    private Boolean savedWarpSpaceEnabled = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param pidDrive specifies the PID drive object to be used for PID controlled drive.
     * @param delay specifies delay in seconds before PID drive starts. 0 means no delay.
     * @param drivePowerLimit specifies the power limit to be applied for the PID controlled drive.
     * @param useSensorOdometry specifies true to use the sensor odometry, false to use drive base odometry.
     * @param tunePidCoeff specifies PID coefficients for tuning PID controllers, can be null if not in
     *        tune mode.
     * @param pathPoints specifies one or more points on the path.
     */
    public CmdPidDrive(
        TrcDriveBase driveBase, TrcPidDrive pidDrive, double delay, double drivePowerLimit, boolean useSensorOdometry,
        TrcPidController.PidCoefficients tunePidCoeff, TrcPose2D... pathPoints)
    {
        if (pathPoints.length == 0)
        {
            throw new IllegalArgumentException("pathPoints must contain at least one point.");
        }

        tracer.traceInfo(
            moduleName,
            "pidDrive=%s,delay=%.3f,powerLimit=%.1f,useSensorOdometry=%s,tunePidCoeff=%s,path=%s",
            pidDrive, delay, drivePowerLimit, useSensorOdometry, tunePidCoeff, Arrays.toString(pathPoints));
        this.driveBase = driveBase;
        this.pidDrive = pidDrive;
        this.delay = delay;
        this.useSensorOdometry = useSensorOdometry;
        this.tunePidCoeff = tunePidCoeff;
        this.pathPoints = pathPoints;
        pathIndex = 0;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);

        xPidCtrl = pidDrive.getXPidCtrl();
        yPidCtrl = pidDrive.getYPidCtrl();
        turnPidCtrl = pidDrive.getTurnPidCtrl();
        //
        // Set power limits for each direction if applicable.
        //
        if (xPidCtrl != null) xPidCtrl.saveAndSetOutputLimit(drivePowerLimit);
        if (yPidCtrl != null) yPidCtrl.saveAndSetOutputLimit(drivePowerLimit);
        if (turnPidCtrl != null) turnPidCtrl.saveAndSetOutputLimit(drivePowerLimit);

        sm.start(State.DO_DELAY);
    }   //CmdPidDrive

    /**
     * Constructor: Create an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param pidDrive specifies the PID drive object to be used for PID controlled drive.
     * @param delay specifies delay in seconds before PID drive starts. 0 means no delay.
     * @param drivePowerLimit specifies the power limit to be applied for the PID controlled drive.
     * @param tunePidCoeff specifies PID coefficients for tuning PID controllers, can be null if not in
     *        tune mode.
     * @param pathPoints specifies one or more points on the path.
     */
    public CmdPidDrive(
        TrcDriveBase driveBase, TrcPidDrive pidDrive, double delay, double drivePowerLimit,
        TrcPidController.PidCoefficients tunePidCoeff, TrcPose2D... pathPoints)
    {
        this(driveBase, pidDrive, delay, drivePowerLimit, false, tunePidCoeff, pathPoints);
    }   //CmdPidDrive

    /**
     * Constructor: Create an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param pidDrive specifies the PID drive object to be used for PID controlled drive.
     * @param delay specifies delay in seconds before PID drive starts. 0 means no delay.
     * @param useSensorOdometry specifies true to use the sensor odometry, false to use drive base odometry.
     *        tune mode.
     * @param pathPoints specifies one or more points on the path.
     */
    public CmdPidDrive(
        TrcDriveBase driveBase, TrcPidDrive pidDrive, double delay, boolean useSensorOdometry, TrcPose2D... pathPoints)
    {
        this(driveBase, pidDrive, delay, 1.0, useSensorOdometry, null, pathPoints);
    }   //CmdPidDrive

    /**
     * Constructor: Create an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param pidDrive specifies the PID drive object to be used for PID controlled drive.
     * @param delay specifies delay in seconds before PID drive starts. 0 means no delay.
     * @param pathPoints specifies one or more points on the path.
     */
    public CmdPidDrive(
        TrcDriveBase driveBase, TrcPidDrive pidDrive, double delay, TrcPose2D... pathPoints)
    {
        this(driveBase, pidDrive, delay, 1.0, false, null, pathPoints);
    }   //CmdPidDrive

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        if (pidDrive.isActive())
        {
            pidDrive.cancel();
        }

        if (xPidCtrl != null) xPidCtrl.restoreOutputLimit();
        if (yPidCtrl != null) yPidCtrl.restoreOutputLimit();
        if (turnPidCtrl != null) turnPidCtrl.restoreOutputLimit();

        if (savedPidCoeff != null)
        {
            tunePidCtrl.setPidCoefficients(savedPidCoeff);
            savedPidCoeff = null;
        }

        if (savedTargetIsAbsolute != null)
        {
            tunePidCtrl.setAbsoluteSetPoint(savedTargetIsAbsolute);
            savedTargetIsAbsolute = null;
        }

        if (savedWarpSpaceEnabled != null)
        {
            pidDrive.setWarpSpaceEnabled(savedWarpSpaceEnabled);
            savedWarpSpaceEnabled = null;
        }

        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            dashboard.displayPrintf(1, "State: " + state);
            tracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.PREP_FOR_TUNING);
                        //
                        // Intentionally falling through to DO_PID_DRIVE.
                        //
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.PREP_FOR_TUNING);
                        break;
                    }

                case PREP_FOR_TUNING:
                    pidDrive.resetAbsoluteTargetPose();
                    if (tunePidCoeff != null)
                    {
                        //
                        // We are in tune mode to tune PID. We tune PID one direction at a time and we only use the
                        // first point of the path.
                        // Read PID constants from the robot and change the corresponding PID controller with them.
                        //
                        if (pathPoints[0].x != 0.0 && (tunePidCtrl = xPidCtrl) != null ||
                            pathPoints[0].y != 0.0 && (tunePidCtrl = yPidCtrl) != null ||
                            pathPoints[0].angle != 0.0 && (tunePidCtrl = turnPidCtrl) != null)
                        {
                            savedPidCoeff = tunePidCtrl.getPidCoefficients();
                            savedTargetIsAbsolute = tunePidCtrl.hasAbsoluteSetPoint();

                            tunePidCtrl.setPidCoefficients(tunePidCoeff);
                            tunePidCtrl.setAbsoluteSetPoint(false);
                            tracer.traceInfo(moduleName, tunePidCtrl + ": PidCoeff=" + tunePidCoeff);
                        }
                        //
                        // Do not optimize turning if we are tuning PID.
                        //
                        savedWarpSpaceEnabled = pidDrive.isWarpSpaceEnabled();
                        pidDrive.setWarpSpaceEnabled(false);
                    }
                    sm.setState(State.PID_DRIVE);
                    //
                    // Intentionally falling through.
                    //
                case PID_DRIVE:
                    //
                    // Drive the set distance and heading.
                    //
                    if (pathIndex < pathPoints.length)
                    {
                        State nextState;

                        if (useSensorOdometry)
                        {
                            // When doing a sensor target, we can only do one path point.
                            pidDrive.setSensorTarget(
                                pathPoints[pathIndex].x, pathPoints[pathIndex].y, pathPoints[pathIndex].angle, event);
                            nextState = State.DONE;
                        }
                        else
                        {
                            // If we are tuning PID, we are doing only one path point.
                            pidDrive.setRelativeTarget(
                                pathPoints[pathIndex].x, pathPoints[pathIndex].y, pathPoints[pathIndex].angle, event);
                            nextState = tunePidCoeff != null? State.DONE: State.PID_DRIVE;
                            pathIndex++;
                        }
                        sm.waitForSingleEvent(event, nextState);
                    }
                    else
                    {
                        //
                        // We ran out of path points, so we will quit.
                        //
                        sm.setState(State.DONE);
                    }
                    break;

                case DONE:
                default:
                    //
                    // We are done, restore everything.
                    //
                    cancel();
                    break;
            }
            tracer.tracePostStateInfo(sm.toString(), state, driveBase, pidDrive);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdPidDrive
