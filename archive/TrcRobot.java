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

package trclib.archive;

/**
 * This class is a container. It contains some robot related types and interfaces.
 */
public class TrcRobot
{
    public enum RunMode
    {
        /**
         * The robot is only in this mode very briefly during initialization.
         */
        INVALID_MODE,

        /**
         * In FRC, the robot is in this mode before competition starts and between mode transitions from Autonomous
         * to TeleOp, for example. (i.e. DISABLED_MODE to AUTO_MODE to DISABLED_MODE to TELEOP_MODE to DISABLED_MODE).
         * This mode does not exist in FTC.
         */
        DISABLED_MODE,

        /**
         * The robot is in this mode during the autonomous period.
         */
        AUTO_MODE,

        /**
         * The robot is in this mode during the operator control period.
         */
        TELEOP_MODE,

        /**
         * The robot is in this mode when Test Mode is selected on the DriverStation.
         */
        TEST_MODE

    }   //enum RunMode

    /**
     * This interface is normally implemented by the main robot code. The methods in this interface is called by
     * the Cooperative Multitasking Scheduler.
     */
    public interface RobotMode
    {
        /**
         * This method is called on the main robot thread when the competition mode is about to start. In FTC, this
         * is called when the "Play" button on the Driver Station is pressed. Typically, you put code that will
         * prepare the robot for start of competition here such as resetting the encoders/sensors and enabling some
         * sensors to start sampling.
         *
         * @param prevMode specifies the previous RunMode it is coming from.
         * @param nextMode specifies the next RunMode it is going into.
         */
        void startMode(RunMode prevMode, RunMode nextMode);

        /**
         * This method is called on the main robot thread when competition mode is about to end. Typically, you put
         * code that will do clean up here such as disabling the sampling of some sensors.
         *
         * @param prevMode specifies the previous RunMode it is coming from.
         * @param nextMode specifies the next RunMode it is going into.
         */
        void stopMode(RunMode prevMode, RunMode nextMode);

        /**
         * This method is called on the main robot thread periodically at PERIODIC_INTERVAL. Typically, you put the
         * competition code here that processes sensor/gamepad inputs and controls the movement of the robot. Code
         * that doesn't require high frequency processing should observe the slowPeriodicLoop parameter and only
         * runs when it is true (e.g. TeleOp control and robot status updates).
         *
         * @param elapsedTime specifies the elapsed time since the mode started.
         * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
         *        false otherwise.
         */
        void periodic(double elapsedTime, boolean slowPeriodicLoop);

    }   //interface RobotMode

    /**
     * This interface is used to implement a robot command. A robot command consists of a sequence of actions
     * typically executed using a state machine. It can be used to implement an entire autonomous strategy or
     * it can implement a subset of an autonomous strategy that can be reused as a common portion of multiple
     * autonomous strategies. This helps to reduce code duplication and improves code maintenance.
     */
    public interface RobotCommand
    {
        /**
         * This method is call periodically to perform the robot command. Typically, it is implemented by using
         * a state machine.
         *
         * @param elapsedTime specifies the elapsed time of the period in seconds.
         * @return true if the command is done, false otherwise.
         */
        boolean cmdPeriodic(double elapsedTime);

        /**
         * This method checks if the current RobotCommand is running.
         *
         * @return True if the command is running, false otherwise.
         */
        boolean isActive();

        /**
         * This method is called to cancel the RobotCommand prematurely.
         */
        void cancel();

    }   //interface RobotCommand

    private static RunMode currRunMode = RunMode.INVALID_MODE;

    /**
     * This method is used by the robot scheduler to set the current run mode.
     *
     * @param runMode specifies the current robot RunMode.
     */
    public static void setRunMode(RunMode runMode)
    {
        currRunMode = runMode;
    }   //setRunMode

    /**
     * This method returns the current robot RunMode.
     * @return robot RunMode.
     */
    public static RunMode getRunMode()
    {
        return currRunMode;
    }   //getRunMode

}   //class TrcRobot
