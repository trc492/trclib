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

import trclib.archive.TrcDashboard;
import trclib.archive.TrcDbgTrace;
import trclib.archive.TrcDriveBase;
import trclib.archive.TrcEvent;
import trclib.archive.TrcPidDrive;
import trclib.archive.TrcPidDrive.TurnMode;
import trclib.archive.TrcRobot;
import trclib.archive.TrcStateMachine;

/**
 * This class implements a waltz turn command sequence. It is useful for avoiding a pushing match with our
 * opponent. If our opponent is trying to engage a pushing match with us, the driver can push a button and
 * cause the robot to pivot left or right 180 degrees and continue on with the robot driving in reverse.
 */
public class CmdWaltzTurn implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdWaltzTurn.class.getSimpleName();

    private static enum State
    {
        WALTZ_TURN,
        DONE
    }   //enum State

    private final TrcDashboard dashboard = TrcDashboard.getInstance();
    private final TrcDbgTrace tracer = new TrcDbgTrace();
    private final TrcDriveBase driveBase;
    private final TrcPidDrive pidDrive;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private TurnMode prevTurnMode;
    private boolean clockwiseTurn = false;
    private boolean driveInverted = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param pidDrive specifies the PID drive object to be used for PID controlled drive.
     */
    public CmdWaltzTurn(TrcDriveBase driveBase, TrcPidDrive pidDrive)
    {
        this.driveBase = driveBase;
        this.pidDrive = pidDrive;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        prevTurnMode = pidDrive.getTurnMode();
    }   //CmdWaltzTurn

    /**
     * This method is called to start the command sequence doing a clockwise or counter-clockwise waltz turn.
     *
     * @param clockwiseTurn specifies true for a clockwise turn, false for counter-clockwise turn.
     * @param driveInverted specifies the current inverted drive state of the robot.
     */
    public void start(boolean clockwiseTurn, boolean driveInverted)
    {
        this.clockwiseTurn = clockwiseTurn;
        this.driveInverted = driveInverted;
        cancel();
        sm.start(State.WALTZ_TURN);
    }   //start

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
            pidDrive.setTurnMode(prevTurnMode);
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
            double turnTarget = 0.0;

            dashboard.displayPrintf(1, "State: " + state);
            tracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case WALTZ_TURN:
                    //
                    // Do the waltz turn.
                    //
                    turnTarget = clockwiseTurn? 180.0: -180.0;
                    prevTurnMode = pidDrive.getTurnMode();
                    pidDrive.setTurnMode(driveInverted? TurnMode.PIVOT_FORWARD: TurnMode.PIVOT_BACKWARD);
                    pidDrive.setRelativeTurnTarget(turnTarget, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    pidDrive.setTurnMode(prevTurnMode);
                    sm.stop();
                    break;
            }
            tracer.tracePostStateInfo(sm.toString(), state, driveBase, pidDrive);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdWaltzTurn
