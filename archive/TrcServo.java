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

import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;

import trclib.timer.TrcElapsedTimer;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent servo. Typically, this class is to be extended by a platform
 * dependent servo class and must provide a set of abstract methods. This makes sure the rest of the TrcLib classes
 * can access the servo without any knowledge of platform dependent implementations.
 */
public abstract class TrcServo implements TrcExclusiveSubsystem
{
    /**
     * This method inverts the servo direction.
     *
     * @param inverted specifies true to invert the servo direction, false otherwise.
     */
    public abstract void setInverted(boolean inverted);

    /**
     * This method checks if the servo direction is inverted.
     *
     * @return true if the servo direction is inverted, false otherwise.
     */
    public abstract boolean isInverted();

    /**
     * This method sets the logical position of the servo.
     *
     * @param position specifies the logical position of the servo in the range of [0.0, 1.0].
     */
    public abstract void setLogicalPosition(double position);

    /**
     * This method returns the logical position of the servo. In general, servo do not provide real time position
     * feedback. Therefore, it will return the position set by the last setLogicalPosition call.
     *
     * @return logical position of the servo in the range of [0.0, 1.0].
     */
    public abstract double getLogicalPosition();

    private enum ActionType
    {
        SetPosition,
        SetPositionWithStepRate,
        SetPower,
    }   //enum ActionType

    private static class ActionParams
    {
        ActionType actionType = null;
        double power = 0.0;
        double currPosition = 0.0;
        double targetPosition = 0.0;
        double currStepRate = 0.0;
        TrcEvent completionEvent = null;
        double timeout = 0.0;
        double prevTime = 0.0;

        void setPositionParams(double targetPos, TrcEvent completionEvent, double timeout)
        {
            this.actionType = ActionType.SetPosition;
            this.targetPosition = targetPos;
            this.completionEvent = completionEvent;
            this.timeout = timeout;
        }   //setPositionParams

        void setPositionWithStepRateParams(double targetPos, double stepRate, TrcEvent completionEvent)
        {
            this.actionType = ActionType.SetPositionWithStepRate;
            this.targetPosition = targetPos;
            this.currStepRate = stepRate;
            this.completionEvent = completionEvent;
        }   //setPositionWithStepRateParams

        void setPowerParams(double power)
        {
            this.actionType = ActionType.SetPower;
            this.power = power;
            this.targetPosition = 0.0;
            this.currStepRate = 0.0;
        }   //setPowerParams

        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "(action=%s, power=%f, currPos=%f, targetPos=%f, currStepRate=%f, event=%s, timeout=%f)",
                actionType, power, currPosition, targetPosition, currStepRate, completionEvent, timeout);
        }   //toString
    }   //class ActionParams

    private static final double DEF_PHYSICAL_MIN    = 0.0;
    private static final double DEF_PHYSICAL_MAX    = 1.0;
    private static final double DEF_LOGICAL_MIN     = 0.0;
    private static final double DEF_LOGICAL_MAX     = 1.0;
    protected static TrcElapsedTimer servoSetPosElapsedTimer = null;
    private final ArrayList<TrcServo> followers = new ArrayList<>();

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcTimer timer;
    private final TrcTaskMgr.TaskObject servoTaskObj;
    private final AtomicReference<ActionParams> currActionParams = new AtomicReference<>();

    private double physicalMin = DEF_PHYSICAL_MIN;
    private double physicalMax = DEF_PHYSICAL_MAX;
    private double logicalMin = DEF_LOGICAL_MIN;
    private double logicalMax = DEF_LOGICAL_MAX;
    private double maxStepRate = 0.0;
    private double minPos = 0.0;
    private double maxPos = 1.0;
    private double currPower = 0.0;
    private double presetTolerance = 0.0;
    private double[] posPresets = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name of the servo.
     */
    public TrcServo(String instanceName)
    {
        tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        timer = new TrcTimer(instanceName);
        servoTaskObj = TrcTaskMgr.createTask(instanceName + ".servoTask", this::servoTask);
        TrcTaskMgr.createTask(instanceName + ".stopTask", this::stopTask).registerTask(TrcTaskMgr.TaskType.STOP_TASK);
    }   //TrcServo

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
     * This method sets the message tracer for logging trace messages.
     *
     * @param msgLevel specifies the message level.
     */
    public void setTraceLevel(TrcDbgTrace.MsgLevel msgLevel)
    {
        tracer.setTraceLevel(msgLevel);
    }   //setTraceLevel

    /**
     * This method sets the logical range of the servo motor. This is typically used to limit the logical range
     * of the servo to less than the 0.0 to 1.0 range. For example, one may limit the logical range to 0.2 to 0.8.
     *
     * @param logicalMin specifies the minimum value of the logical range.
     * @param logicalMax specifies the maximum value of the logical range.
     */
    public void setLogicalPosRange(double logicalMin, double logicalMax)
    {
        if (logicalMin >= logicalMax)
        {
            throw new IllegalArgumentException("max must be greater than min.");
        }

        this.logicalMin = logicalMin;
        this.logicalMax = logicalMax;
    }   //setLogicalPosRange

    /**
     * This method sets the physical range of the servo motor. This is typically used to set a 180-degree servo to
     * have a range of 0.0 to 180.0 instead of the logical range of 0.0 to 1.0. By default physical range is set
     * to the range of 0.0 to 1.0, same as logical range. The physical range is used for map physical position
     * units to logical position unit between 0.0 to 1.0.
     *
     * @param physicalMin specifies the minimum value of the physical range.
     * @param physicalMax specifies the maximum value of the physical range.
     */
    public void setPhysicalPosRange(double physicalMin, double physicalMax)
    {
        if (physicalMin >= physicalMax)
        {
            throw new IllegalArgumentException("max must be greater than min.");
        }

        this.physicalMin = physicalMin;
        this.physicalMax = physicalMax;
    }   //setPhysicalPosRange

    /**
     * This method is called to convert a physical position to a logical position. It will make sure the physical
     * position is within the physical range and scale it to the logical range.
     *
     * @param physicalPosition specifies the physical position to be converted
     * @return converted logical position.
     */
    public double toLogicalPosition(double physicalPosition)
    {
        physicalPosition = TrcUtil.clipRange(physicalPosition, physicalMin, physicalMax);
        return TrcUtil.scaleRange(physicalPosition, physicalMin, physicalMax, logicalMin, logicalMax);
    }   //toLogicalPosition

    /**
     * This method is called to convert a logical position to a physical position.
     * It will make sure the logical position is within the logical range and scale
     * it to the physical range.
     * Note: this method is only callable by classes extending this class.
     *
     * @param logicalPosition specifies the logical position to be converted.
     * @return converted physical position.
     */
    public double toPhysicalPosition(double logicalPosition)
    {
        logicalPosition = TrcUtil.clipRange(logicalPosition, logicalMin, logicalMax);
        return TrcUtil.scaleRange(logicalPosition, logicalMin, logicalMax, physicalMin, physicalMax);
    }   //toPhysicalPosition

    /**
     * This method sets the stepping mode parameters.
     *
     * @param maxStepRate specifies the maximum stepping rate (physicalPos/sec).
     * @param minPos specifies the minimum physical position.
     * @param maxPos specifies the maximum physical position.
     */
    public void setStepModeParams(double maxStepRate, double minPos, double maxPos)
    {
        tracer.traceDebug(
            instanceName, "maxStepRate=%f, minPos=%f, maxPos=%f", maxStepRate, minPos, maxPos);
        this.maxStepRate = maxStepRate;
        this.minPos = minPos;
        this.maxPos = maxPos;
    }   //setStepModeParams

    /**
     * This method adds a following servo to the followers list.
     *
     * @param followingServo specifies the following servo.
     * @return true if the servo is added successfully, false if it is already in the list.
     */
    private boolean addFollower(TrcServo followingServo)
    {
        boolean success = false;

        synchronized (followers)
        {
            if (!followers.contains(followingServo))
            {
                success = followers.add(followingServo);
            }
        }

        return success;
    }   //addFollower

    /**
     * This method sets this servo to follow the specified servo.
     *
     * @param servo specifies the other servo to follow.
     */
    public void follow(TrcServo servo)
    {
        servo.addFollower(this);
    }   //follow

    /**
     * This method finishes previous servo operation if applicable.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param canceled specifies true if the operation was canceled.
     */
    private void finish(String owner, boolean canceled)
    {
        ActionParams actionParams = currActionParams.getAndSet(null);

        tracer.traceDebug(
            instanceName,
            "owner=" + owner +
            ", canceled=" + canceled +
            ", actionParams=" + actionParams +
            ", taskEnabled=" + isTaskEnabled());
        if (actionParams != null && validateOwnership(owner))
        {
            synchronized (actionParams)
            {
                if (canceled)
                {
                    timer.cancel();
                }

                if (isTaskEnabled())
                {
                    setTaskEnabled(false);
                }

                if (actionParams.completionEvent != null)
                {
                    if (canceled)
                    {
                        actionParams.completionEvent.cancel();
                    }
                    else
                    {
                        actionParams.completionEvent.signal();
                    }
                    actionParams.completionEvent = null;
                }
            }
        }
    }   //finish

    /**
     * This method finishes previous servo operation if applicable.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     */
    public void cancel(String owner)
    {
        finish(owner, true);
    }   //cancel

    /**
     * This method cancels previous servo operation if applicable.
     */
    public void cancel()
    {
        finish(null, true);
    }   //cancel

    /**
     * This method returns the physical position value of the servo. Generally, servo do not provide real time position
     * feedback. Therefore, it will only return the position set by the last setPosition call.
     *
     * @return physical position of the servo, could be in degrees if setPhysicalPosRange is called to set the range in
     *         degrees.
     */
    public double getPosition()
    {
        return toPhysicalPosition(getLogicalPosition());
    }   //getPosition

    /**
     * This method performs the setPosition action.
     *
     * @param context specifies the action params.
     */
    private void performSetPosition(Object context)
    {
        ActionParams actionParams = (ActionParams) context;

        synchronized (actionParams)
        {
            double logicalPos =
                toLogicalPosition(actionParams.actionType == ActionType.SetPosition ?
                                      actionParams.targetPosition : actionParams.currPosition);

            tracer.traceDebug(instanceName, "actionParams=%s, logicalPos=%f", actionParams, logicalPos);
            setLogicalPosition(logicalPos);
            synchronized (followers)
            {
                for (TrcServo servo : followers)
                {
                    servo.setLogicalPosition(logicalPos);
                }
            }

            if (actionParams.timeout > 0.0 && actionParams.completionEvent != null)
            {
                // A timeout is specified, set a timer for it and signal an event when it expires.
                // Since servo has no position feedback mechanism, time is used to estimate how long it takes to
                // complete the operation and signal the caller.
                timer.set(actionParams.timeout, actionParams.completionEvent);
                actionParams.completionEvent = null;
            }
            // Action performed, destroy the params if ActionType was SetPosition. SetPositionWithStepRate and SetPower
            // will take care of cleaning up itself.
            if (actionParams.actionType == ActionType.SetPosition)
            {
                finish(null, false);
            }
        }
    }   //performSetPosition

    /**
     * This method sets the servo position. By default, the servo maps its physical position the same as its logical
     * position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world physical range
     * (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets event after
     * the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(
        String owner, double delay, double position, TrcEvent completionEvent, double timeout)
    {
        tracer.traceDebug(
            instanceName, "owner=%s, delay=%f, pos=%f, event=%s, timeout=%f",
            owner, delay, position, completionEvent, timeout);
        if (completionEvent != null)
        {
            completionEvent.clear();
        }

        finish(owner, true);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            ActionParams actionParams = new ActionParams();
            actionParams.setPositionParams(position, completionEvent, timeout);
            if (currActionParams.compareAndSet(null, actionParams))
            {
                if (delay > 0.0)
                {
                    timer.set(delay, this::performSetPosition, actionParams);
                }
                else
                {
                    performSetPosition(actionParams);
                }
            }
            else
            {
                throw new IllegalStateException("Existing active action (" + currActionParams.get() + ")");
            }
        }
    }   //setPosition

    /**
     * This method sets the servo position. By default, the servo maps its physical position the same as its logical
     * position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world physical range
     * (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets event after
     * the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param delay speciifes the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(double delay, double position, TrcEvent completionEvent, double timeout)
    {
        setPosition(null, delay, position, completionEvent, timeout);
    }   //setPosition

    /**
     * This method sets the servo position. By default, the servo maps its physical position the same as its logical
     * position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world physical range
     * (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets event after
     * the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(double position, TrcEvent completionEvent, double timeout)
    {
        setPosition(null, 0.0, position, completionEvent, timeout);
    }   //setPosition

    /**
     * This method sets the servo position. By default, the servo maps its physical position the same as its logical
     * position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world physical range
     * (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets event after
     * the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     */
    public void setPosition(double position)
    {
        setPosition(null, 0.0, position, null, 0.0);
    }   //setPosition

    /**
     * This method performs the setPositionWithStepRate action.
     *
     * @param context specifies the action params.
     */
    private void performSetPositionWithStepRate(Object context)
    {
        ActionParams actionParams = (ActionParams) context;

        synchronized (actionParams)
        {
            actionParams.currPosition = getPosition();
            actionParams.prevTime = TrcTimer.getCurrentTime();
            setTaskEnabled(true);
            tracer.traceDebug(instanceName, "actionParams=" + actionParams);
        }
    }   //performSetPositionWithStepRate

    /**
     * This method sets the servo to the specifies position but with the specified step rate in effect controlling
     * the speed to get there.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay speciifes the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the target position.
     * @param stepRate specifies the stepping rate to get there (physicalPos/sec).
     * @param completionEvent specifies an event object to signal when the servo reaches the position..
     */
    public void setPosition(
        String owner, double delay, double position, double stepRate, TrcEvent completionEvent)
    {
        tracer.traceDebug(
            instanceName, "owner=%s, delay=%f, pos=%f, stepRate=%f, event=%s",
            owner, delay, position, stepRate, completionEvent);
        if (completionEvent != null)
        {
            completionEvent.clear();
        }

        finish(owner, true);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            ActionParams actionParams = new ActionParams();
            actionParams.setPositionWithStepRateParams(position, stepRate, completionEvent);
            if (currActionParams.compareAndSet(null, actionParams))
            {
                if (delay > 0.0)
                {
                    timer.set(delay, this::performSetPositionWithStepRate, actionParams);
                }
                else
                {
                    performSetPositionWithStepRate(actionParams);
                }
            }
            else
            {
                throw new IllegalStateException("Existing active action (" + currActionParams.get() + ")");
            }
        }
    }   //setPosition

    /**
     * This method sets the servo to the specifies position but with the specified step rate in effect controlling
     * the speed to get there.
     *
     * @param delay speciifes the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the target position.
     * @param stepRate specifies the stepping rate to get there (physicalPos/sec).
     * @param completionEvent specifies an event object to signal when the servo reaches the position..
     */
    public void setPosition(double delay, double position, double stepRate, TrcEvent completionEvent)
    {
        setPosition(null, delay, position, stepRate, completionEvent);
    }   //setPosition

    /**
     * This method sets the servo to the specifies position but with the specified step rate in effect controlling
     * the speed to get there.
     *
     * @param position specifies the target position.
     * @param stepRate specifies the stepping rate to get there (physicalPos/sec).
     * @param completionEvent specifies an event object to signal when the servo reaches the position..
     */
    public void setPosition(double position, double stepRate, TrcEvent completionEvent)
    {
        setPosition(null, 0.0, position, stepRate, completionEvent);
    }   //setPosition

    /**
     * This method sets the servo to the specifies position but with the specified step rate in effect controlling
     * the speed to get there.
     *
     * @param position specifies the target position.
     * @param stepRate specifies the stepping rate to get there (physicalPos/sec).
     */
    public void setPosition(double position, double stepRate)
    {
        setPosition(null, 0.0, position, stepRate, null);
    }   //setPosition

    /**
     * This method performs the setPower action.
     *
     * @param context not used.
     */
    private void performSetPower(Object context)
    {
        ActionParams actionParams = (ActionParams) context;

        synchronized (actionParams)
        {
            if (!isTaskEnabled())
            {
                // Not already in stepping mode, do a setPosition to the direction according to the sign of the power
                // and the step rate according to the magnitude of the power.
                actionParams.targetPosition = actionParams.power > 0.0 ? maxPos : minPos;
                actionParams.currStepRate = Math.abs(actionParams.power)*maxStepRate;
                actionParams.currPosition = getPosition();
                actionParams.prevTime = TrcTimer.getCurrentTime();
                setTaskEnabled(true);
            }
            else if (actionParams.power != 0.0)
            {
                // We are already in stepping mode, just change the stepping parameters.
                actionParams.targetPosition = actionParams.power > 0.0 ? maxPos : minPos;
                actionParams.currStepRate = Math.abs(actionParams.power)*maxStepRate;
            }
            else
            {
                // We are stopping.
                finish(null, false);
            }
            currPower = actionParams.power;
            tracer.traceDebug(instanceName, "actionParams=" + actionParams + ", taskEnabled=" + isTaskEnabled());
        }
    }   //performSetPower

    /**
     * This method sets power of the continuous servo.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay speciifes the delay in seconds before setting the power of the servo, can be zero if no delay.
     * @param power specifies how fast the servo will turn.
     */
    public void setPower(String owner, double delay, double power)
    {
        tracer.traceDebug(instanceName, "owner=%s, delay=%f, power=%f", owner, delay, power);
        if (validateOwnership(owner))
        {
            ActionParams actionParams = new ActionParams();
            actionParams.setPowerParams(power);
            currActionParams.set(actionParams);
            if (delay > 0.0)
            {
                timer.set(delay, this::performSetPower, actionParams);
            }
            else
            {
                performSetPower(actionParams);
            }
        }
    }   //setPower

    /**
     * This method sets power of the continuous servo.
     *
     * @param delay speciifes the delay in seconds before setting the power of the servo, can be zero if no delay.
     * @param power specifies how fast the servo will turn.
     */
    public void setPower(double delay, double power)
    {
        setPower(null, delay, power);
    }   //setPower

    /**
     * This method sets power of the continuous servo.
     *
     * @param power specifies how fast the servo will turn.
     */
    public void setPower(double power)
    {
        setPower(null, 0.0, power);
    }   //setPower

    /**
     * This method returns the last set power value.
     *
     * @return last power set to the motor.
     */
    public double getPower()
    {
        return currPower;
    }   //getPower

    /**
     * This method enables/disables the enhanced servo task for performing step rate speed control or zero
     * calibration.
     *
     * @param enabled specifies true to enable task, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            servoTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
        }
        else
        {
            servoTaskObj.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This method checks if the task is enabled.
     *
     * @return true if task is enabled, false otherwise.
     */
    private boolean isTaskEnabled()
    {
        return servoTaskObj.isRegistered();
    }   //isTaskEnabled

    /**
     * This method is called periodically to check whether the servo has reached target. If not, it will calculate
     * the next position to set the servo to according to its step rate.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void servoTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        ActionParams actionParams = currActionParams.get();

        if (actionParams != null)
        {
            synchronized (actionParams)
            {
                double currTime = TrcTimer.getCurrentTime();
                double deltaPos = actionParams.currStepRate*(currTime - actionParams.prevTime);

                actionParams.prevTime = currTime;
                if (actionParams.currPosition < actionParams.targetPosition)
                {
                    actionParams.currPosition += deltaPos;
                    if (actionParams.currPosition > actionParams.targetPosition)
                    {
                        actionParams.currPosition = actionParams.targetPosition;
                    }
                }
                else if (actionParams.currPosition > actionParams.targetPosition)
                {
                    actionParams.currPosition -= deltaPos;
                    if (actionParams.currPosition < actionParams.targetPosition)
                    {
                        actionParams.currPosition = actionParams.targetPosition;
                    }
                }
                else
                {
                    //
                    // We have reached target, we are done.
                    //
                    finish(null, false);
                }

                tracer.traceVerbose(instanceName, "actionParams=" + actionParams);
                performSetPosition(actionParams);
            }
        }
    }   //servoTask

    /**
     * This method is called when the competition mode is about to end so it will stop the servo if necessary.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void stopTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        cancel(null);
    }   //stopTask

    //
    // Position presets.
    //

    /**
     * This method sets an array of preset positions for the motor.
     *
     * @param tolerance specifies the preset tolerance.
     * @param posPresets specifies an array of preset positions in scaled unit.
     */
    public void setPosPresets(double tolerance, double... posPresets)
    {
        this.presetTolerance = tolerance;
        this.posPresets = posPresets;
    }   //setPosPresets

    /**
     * This method checks if the preset index is within the preset table.
     *
     * @param index specifies the preset table index to check.
     * @return true if there is a preset table and the index is within the table.
     */
    public boolean validatePresetIndex(int index)
    {
        return posPresets != null && index >= 0 && index < posPresets.length;
    }   //validatePresetIndex

    /**
     * This method returns the preset position at the specified index.
     *
     * @param index specifies the index into the preset position table.
     * @return preset position.
     */
    public double getPresetPosition(int index)
    {
        return posPresets[index];
    }   //getPresetPosition

    /**
     * This method sets the servo to the specified preset position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param presetIndex specifies the index to the preset position array.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *        timeout, the operation will be canceled and the event will be signaled. If no timeout is specified, it
     *        should be set to zero.
     */
    public void setPresetPosition(
        String owner, double delay, int presetIndex, TrcEvent event, double timeout)
    {
        if (validatePresetIndex(presetIndex))
        {
            setPosition(owner, delay, posPresets[presetIndex], event, timeout);
        }
    }   //setPresetPosition

    /**
     * This method sets the servo to the specified preset position.
     *
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param preset specifies the index to the preset position array.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *        timeout, the operation will be canceled and the event will be signaled. If no timeout is specified, it
     *        should be set to zero.
     */
    public void setPresetPosition(double delay, int preset, TrcEvent event, double timeout)
    {
        setPresetPosition(null, delay, preset, event, timeout);
    }   //setPresetPosition

    /**
     * This method sets the servo to the specified preset position.
     *
     * @param preset specifies the index to the preset position array.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *        timeout, the operation will be canceled and the event will be signaled. If no timeout is specified, it
     *        should be set to zero.
     */
    public void setPresetPosition(int preset, TrcEvent event, double timeout)
    {
        setPresetPosition(null, 0.0, preset, event, timeout);
    }   //setPresetPosition

    /**
     * This method sets the servo to the specified preset position.
     *
     * @param preset specifies the index to the preset position array.
     * @param event specifies the event to signal when done, can be null if not provided.
     */
    public void setPresetPosition(int preset, TrcEvent event)
    {
        setPresetPosition(null, 0.0, preset, event, 0.0);
    }   //setPresetPosition

    /**
     * This method sets the servo to the specified preset position.
     *
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param preset specifies the index to the preset position array.
     */
    public void setPresetPosition(double delay, int preset)
    {
        setPresetPosition(null, delay, preset, null, 0.0);
    }   //setPresetPosition

    /**
     * This method sets the servo to the specified preset position.
     *
     * @param preset specifies the index to the preset position array.
     */
    public void setPresetPosition(int preset)
    {
        setPresetPosition(null, 0.0, preset, null, 0.0);
    }   //setPresetPosition

    /**
     * This method determines the next preset index up from the current position.
     *
     * @return next preset index up, -1 if there is no preset table.
     */
    public int nextPresetIndexUp()
    {
        int index = -1;

        if (posPresets != null)
        {
            double currPos = getPosition();

            for (int i = 0; i < posPresets.length; i++)
            {
                if (posPresets[i] > currPos)
                {
                    index = i;
                    if (Math.abs(currPos - posPresets[i]) <= presetTolerance)
                    {
                        index++;
                    }
                    break;
                }
            }

            if (index == -1)
            {
                index = posPresets.length - 1;
            }
        }

        return index;
    }   //nextPresetIndexUp

    /**
     * This method determines the next preset index down from the current position.
     *
     * @return next preset index down, -1 if there is no preset table.
     */
    public int nextPresetIndexDown()
    {
        int index = -1;

        if (posPresets != null)
        {
            double currPos = getPosition();

            for (int i = posPresets.length - 1; i >= 0; i--)
            {
                if (posPresets[i] < currPos)
                {
                    index = i;
                    if (Math.abs(currPos - posPresets[i]) <= presetTolerance)
                    {
                        index--;
                    }
                    break;
                }
            }

            if (index == -1)
            {
                index = 0;
            }
        }

        return index;
    }   //nextPresetIndexDown

    /**
     * This method sets the actuator to the next preset position up or down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is coompleted, can be null if no ownership
     *        is required.
     * @param presetUp specifies true to move to next preset up, false to move to next preset down.
     */
    private void setNextPresetPosition(String owner, boolean presetUp)
    {
        int index = presetUp? nextPresetIndexUp(): nextPresetIndexDown();

        if (index != -1)
        {
            setPresetPosition(owner, 0.0, index, null, 0.0);
        }
    }   //setNextPresetPosition

    /**
     * This method sets the actuator to the next preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is coompleted, can be null if no ownership
     *        is required.
     */
    public void presetPositionUp(String owner)
    {
        setNextPresetPosition(owner, true);
    }   //presetPositionUp

    /**
     * This method sets the actuator to the next preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is coompleted, can be null if no ownership
     *        is required.
     */
    public void presetPositionDown(String owner)
    {
        setNextPresetPosition(owner, false);
    }   //presetPositionDown

    //
    // Performance monitoring.
    //

    /**
     * This method enables/disables the elapsed timers for performance monitoring.
     *
     * @param enabled specifies true to enable elapsed timers, false to disable.
     */
    public static void setElapsedTimerEnabled(boolean enabled)
    {
        if (enabled)
        {
            if (servoSetPosElapsedTimer == null)
            {
                servoSetPosElapsedTimer = new TrcElapsedTimer("TrcServo.setPos", 2.0);
            }
        }
        else
        {
            servoSetPosElapsedTimer = null;
        }
    }   //setElapsedTimerEnabled

    /**
     * This method prints the elapsed time info.
     *
     * @param msgTracer specifies the tracer to be used to print the info.
     */
    public static void printElapsedTime(TrcDbgTrace msgTracer)
    {
        if (servoSetPosElapsedTimer != null)
        {
            servoSetPosElapsedTimer.printElapsedTime(msgTracer);
        }
    }   //printElapsedTime

}   //class TrcServo
