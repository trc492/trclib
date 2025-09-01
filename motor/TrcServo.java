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

package trclib.motor;

import java.util.ArrayList;
import java.util.Arrays;

import trclib.dataprocessor.TrcUtil;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;
import trclib.robotcore.TrcPresets;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
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

    public static class Params
    {
        public double logicalMin = DEF_LOGICAL_MIN;
        public double logicalMax = DEF_LOGICAL_MAX;
        public double physicalMin = DEF_PHYSICAL_MIN;
        public double physicalMax = DEF_PHYSICAL_MAX;
        public Double maxStepRate = null;
        public double presetTolerance = 0.0;
        public double[] posPresets = null;

        /**
         * This method returns the string format of the servoParams info.
         *
         * @return string format of the servo param info.
         */
        public String toString()
        {
            return "logMin=" + logicalMin +
                   ",logMax=" + logicalMax +
                   ",phyMin=" + physicalMin +
                   ",phyMax=" + physicalMax +
                   ",maxStepRate=" + maxStepRate +
                   ",presetTolerance=" + presetTolerance +
                   ",posPreset=" + Arrays.toString(posPresets);
        }   //toString

        /**
         * This method sets the logical range of the servo motor. This is typically used to limit the logical range
         * of the servo to less than the 0.0 to 1.0 range. For example, one may limit the logical range to 0.2 to 0.8.
         *
         * @param logicalMin specifies the minimum value of the logical range.
         * @param logicalMax specifies the maximum value of the logical range.
         * @return this object for chaining.
         */
        public Params setLogicalPosRange(double logicalMin, double logicalMax)
        {
            this.logicalMin = logicalMin;
            this.logicalMax = logicalMax;
            return this;
        }   //setLogicalPosRange

        /**
         * This method sets the physical range of the servo motor. This is typically used to set a 180-degree servo to
         * have a range of 0.0 to 180.0 instead of the logical range of 0.0 to 1.0. By default physical range is set
         * to the range of 0.0 to 1.0, same as logical range. The physical range is used for map physical position
         * units to logical position unit between 0.0 to 1.0.
         *
         * @param physicalMin specifies the minimum value of the physical range.
         * @param physicalMax specifies the maximum value of the physical range.
         * @return this object for chaining.
         */
        public Params setPhysicalPosRange(double physicalMin, double physicalMax)
        {
            this.physicalMin = physicalMin;
            this.physicalMax = physicalMax;
            return this;
        }   //setPhysicalPosRange

        /**
         * This method sets the maximum stepping rate of the servo. This enables setPower to speed control the servo.
         *
         * @param maxStepRate specifies the maximum stepping rate (physicalPos/sec).
         * @return this object for chaining.
         */
        public Params setMaxStepRate(double maxStepRate)
        {
            this.maxStepRate = maxStepRate;
            return this;
        }   //setMaxStepRate

        /**
         * This method sets an array of preset positions for the servo.
         *
         * @param tolerance specifies the preset tolerance.
         * @param posPresets specifies an array of preset positions in scaled unit.
         * @return this object for chaining.
         */
        public Params setPosPresets(double tolerance, double... posPresets)
        {
            this.presetTolerance = tolerance;
            this.posPresets = posPresets;
            return this;
        }   //setPosPresets

    }   //class Params

    private enum ActionType
    {
        SetPosition,
        SetPositionWithStepRate,
        SetPower
    }   //enum ActionType

    private static class ActionParams
    {
        ActionType actionType = null;
        double power = 0.0;
        double minPos = 0.0;
        double maxPos = 0.0;
        double currPosition = 0.0;
        double targetPosition = 0.0;
        double currStepRate = 0.0;
        TrcEvent completionEvent = null;
        double timeout = 0.0;
        Double prevTime = null;

        @Override
        public String toString()
        {
            return "(action=" + actionType +
                   ",power=" + power +
                   ",minPos=" + minPos +
                   ",maxPos=" + maxPos +
                   ",currPos=" + currPosition +
                   ",targetPos=" + targetPosition +
                   ",currStepRate=" + currStepRate +
                   ",event=" + completionEvent +
                   ",timeout=" + timeout;
        }   //toString

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

        void setPowerParams(double power, double minPos, double maxPos)
        {
            this.actionType = ActionType.SetPower;
            this.power = power;
            this.minPos = minPos;
            this.maxPos = maxPos;
            this.targetPosition = 0.0;
            this.currStepRate = 0.0;
        }   //setPowerParams

    }   //class ActionParams

    private static final double DEF_LOGICAL_MIN     = 0.0;
    private static final double DEF_LOGICAL_MAX     = 1.0;
    private static final double DEF_PHYSICAL_MIN    = 0.0;
    private static final double DEF_PHYSICAL_MAX    = 1.0;
    protected static TrcElapsedTimer servoSetPosElapsedTimer = null;
    private final ArrayList<TrcServo> followers = new ArrayList<>();
    private final ActionParams actionParams = new ActionParams();

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final Params servoParams;
    private final TrcTimer timer;
    private final TrcTaskMgr.TaskObject servoTaskObj;

    private TrcPresets posPresets = null;
    private double currPower = 0.0;
    private Double prevLogicalPos = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name of the servo.
     * @param params specifies the parameters of the servo.
     */
    public TrcServo(String instanceName, Params params)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.servoParams = params != null? params: new Params();
        timer = new TrcTimer(instanceName);
        servoTaskObj = TrcTaskMgr.createTask(instanceName + ".servoTask", this::servoTask);
        TrcTaskMgr.createTask(instanceName + ".stopTask", this::stopTask).registerTask(TrcTaskMgr.TaskType.STOP_TASK);
        if (params != null && params.posPresets != null)
        {
            setPosPresets(params.presetTolerance, params.posPresets);
        }
    }   //TrcServo

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name of the servo.
     */
    public TrcServo(String instanceName)
    {
        this(instanceName, null);
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

        servoParams.setLogicalPosRange(logicalMin, logicalMax);
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

        servoParams.setPhysicalPosRange(physicalMin, physicalMax);
    }   //setPhysicalPosRange

    /**
     * This method sets the maximum stepping rate of the servo. This enables setPower to speed control the servo.
     *
     * @param maxStepRate specifies the maximum stepping rate (physicalPos/sec).
     */
    public void setMaxStepRate(double maxStepRate)
    {
        tracer.traceDebug(instanceName, "maxStepRate=" + maxStepRate);
        servoParams.setMaxStepRate(maxStepRate);
    }   //setMaxStepRate

    /**
     * This method is called to convert a physical position to a logical position. It will make sure the physical
     * position is within the physical range and scale it to the logical range.
     *
     * @param physicalPosition specifies the physical position to be converted
     * @return converted logical position.
     */
    public double toLogicalPosition(double physicalPosition)
    {
        physicalPosition = TrcUtil.clipRange(physicalPosition, servoParams.physicalMin, servoParams.physicalMax);
        return TrcUtil.scaleRange(
            physicalPosition, servoParams.physicalMin, servoParams.physicalMax, servoParams.logicalMin,
            servoParams.logicalMax);
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
        logicalPosition = TrcUtil.clipRange(logicalPosition, servoParams.logicalMin, servoParams.logicalMax);
        return TrcUtil.scaleRange(
            logicalPosition, servoParams.logicalMin, servoParams.logicalMax, servoParams.physicalMin,
            servoParams.physicalMax);
    }   //toPhysicalPosition

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
        tracer.traceDebug(
            instanceName,
            "owner=" + owner +
            ", canceled=" + canceled +
            ", actionParams=" + actionParams +
            ", taskEnabled=" + isTaskEnabled());
        if (validateOwnership(owner))
        {
            synchronized (actionParams)
            {
                if (actionParams.actionType != null)
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

                    actionParams.actionType = null;
                }
            }
        }
    }   //finish

    /**
     * This method cancels previous servo operation if applicable.
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
        double logicalPos = prevLogicalPos != null? prevLogicalPos: getLogicalPosition();
        return toPhysicalPosition(logicalPos);
    }   //getPosition

    /**
     * This method performs the setPosition action.
     *
     * @param context not used.
     * @param canceled specifies true if canceled.
     */
    private void performSetPosition(Object context, boolean canceled)
    {
        if (!canceled)
        {
            synchronized (actionParams)
            {
                double logicalPos =
                    toLogicalPosition(
                        actionParams.actionType == ActionType.SetPosition ?
                            actionParams.targetPosition : actionParams.currPosition);

                tracer.traceDebug(instanceName, "actionParams=" + actionParams + ", logicalPos=" + logicalPos);
                if (prevLogicalPos == null || logicalPos != prevLogicalPos)
                {
                    setLogicalPosition(logicalPos);
                    prevLogicalPos = logicalPos;
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
                        // Since servo has no position feedback mechanism, time is used to estimate how long it takes
                        // to complete the operation and signal the caller.
                        timer.set(actionParams.timeout, actionParams.completionEvent);
                        actionParams.completionEvent = null;
                    }
                }

                // Action performed, destroy the params if ActionType was SetPosition. SetPositionWithStepRate and
                // SetPower will take care of cleaning up itself.
                if (actionParams.actionType == ActionType.SetPosition)
                {
                    finish(null, false);
                }
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
            instanceName,
            "owner=" + owner +
            ", delay=" + delay +
            ", pos=" + position +
            ", event=" + completionEvent +
            ", timeout=" + timeout);
        if (completionEvent != null)
        {
            completionEvent.clear();
        }

        finish(owner, true);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            actionParams.setPositionParams(position, completionEvent, timeout);
            if (delay > 0.0)
            {
                timer.set(delay, this::performSetPosition, null);
            }
            else
            {
                performSetPosition(null, false);
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
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
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
     * @param context not used.
     * @param canceled specifies true if canceled.
     */
    private void performSetPositionWithStepRate(Object context, boolean canceled)
    {
        if (!canceled)
        {
            synchronized (actionParams)
            {
                actionParams.currPosition = getPosition();
                actionParams.prevTime = null;
                setTaskEnabled(true);
                tracer.traceDebug(instanceName, "actionParams=" + actionParams);
            }
        }
    }   //performSetPositionWithStepRate

    /**
     * This method sets the servo to the specifies position but with the specified step rate in effect controlling
     * the speed to get there.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the target position.
     * @param stepRate specifies the stepping rate to get there (physicalPos/sec).
     * @param completionEvent specifies an event object to signal when the servo reaches the position..
     */
    public void setPosition(
        String owner, double delay, double position, double stepRate, TrcEvent completionEvent)
    {
        tracer.traceDebug(
            instanceName,
            "owner=" + owner +
            ", delay=" + delay +
            ", pos=" + position +
            ", stepRate=" + stepRate +
            ", event=" + completionEvent);
        if (completionEvent != null)
        {
            completionEvent.clear();
        }

        finish(owner, true);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            actionParams.setPositionWithStepRateParams(position, stepRate, completionEvent);
            if (delay > 0.0)
            {
                timer.set(delay, this::performSetPositionWithStepRate, null);
            }
            else
            {
                performSetPositionWithStepRate(null, false);
            }
        }
    }   //setPosition

    /**
     * This method sets the servo to the specifies position but with the specified step rate in effect controlling
     * the speed to get there.
     *
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
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
     * @param canceled specifies true if canceled.
     */
    private void performSetPower(Object context, boolean canceled)
    {
        if (!canceled)
        {
            synchronized (actionParams)
            {
                if (!isTaskEnabled())
                {
                    // Not already in stepping mode, do a setPosition to the direction according to the sign of the
                    // power and the step rate according to the magnitude of the power.
                    actionParams.targetPosition = actionParams.power > 0.0? actionParams.maxPos: actionParams.minPos;
                    actionParams.currStepRate = Math.abs(actionParams.power) * servoParams.maxStepRate;
                    actionParams.currPosition = getPosition();
                    actionParams.prevTime = null;
                    setTaskEnabled(true);
                }
                else if (actionParams.power != 0.0)
                {
                    // We are already in stepping mode, just change the stepping parameters.
                    actionParams.targetPosition =
                        actionParams.power > 0.0 ? actionParams.maxPos : actionParams.minPos;
                    actionParams.currStepRate = Math.abs(actionParams.power) * servoParams.maxStepRate;
                }
                else
                {
                    // We are stopping.
                    finish(null, false);
                }
                currPower = actionParams.power;
                tracer.traceDebug(instanceName, "actionParams=" + actionParams + ", taskEnabled=" + isTaskEnabled());
            }
        }
    }   //performSetPower

    /**
     * This method speed controls the servo with stepping.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the power of the servo, can be zero if no delay.
     * @param power specifies how fast the servo will turn.
     * @param minPos specifies the minimum position the servo can move to.
     * @param maxPos specifies the maximum position the servo can move to.
     */
    public void setPower(String owner, double delay, double power, double minPos, double maxPos)
    {
        tracer.traceDebug(
            instanceName, "owner=%s,delay=%.3f,power=%.3f,minPos=%.3f,maxPos=%.3f",
            owner, delay, power, minPos, maxPos);
        if (servoParams.maxStepRate == null)
        {
            throw new IllegalStateException("Maximum stepping rate is not set.");
        }

        if (validateOwnership(owner) && power != currPower)
        {
            actionParams.setPowerParams(power, minPos, maxPos);
            if (delay > 0.0)
            {
                timer.set(delay, this::performSetPower, null);
            }
            else
            {
                performSetPower(null, false);
            }
        }
    }   //setPower

    /**
     * This method speed controls the servo with stepping.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the power of the servo, can be zero if no delay.
     * @param power specifies how fast the servo will turn.
     */
    public void setPower(String owner, double delay, double power)
    {
        setPower(owner, delay, power, servoParams.physicalMin, servoParams.physicalMax);
    }   //setPower

    /**
     * This method speed controls the servo with stepping.
     *
     * @param delay specifies the delay in seconds before setting the power of the servo, can be zero if no delay.
     * @param power specifies how fast the servo will turn.
     * @param minPos specifies the minimum position the servo can move to.
     * @param maxPos specifies the maximum position the servo can move to.
     */
    public void setPower(double delay, double power, double minPos, double maxPos)
    {
        setPower(null, delay, power, minPos, maxPos);
    }   //setPower

    /**
     * This method speed controls the servo with stepping.
     *
     * @param delay specifies the delay in seconds before setting the power of the servo, can be zero if no delay.
     * @param power specifies how fast the servo will turn.
     */
    public void setPower(double delay, double power)
    {
        setPower(null, delay, power, servoParams.physicalMin, servoParams.physicalMax);
    }   //setPower

    /**
     * This method speed controls the servo with stepping.
     *
     * @param power specifies how fast the servo will turn.
     * @param minPos specifies the minimum position the servo can move to.
     * @param maxPos specifies the maximum position the servo can move to.
     */
    public void setPower(double power, double minPos, double maxPos)
    {
        setPower(null, 0.0, power, minPos, maxPos);
    }   //setPower

    /**
     * This method speed controls the servo with stepping.
     *
     * @param power specifies how fast the servo will turn.
     */
    public void setPower(double power)
    {
        setPower(null, 0.0, power, servoParams.physicalMin, servoParams.physicalMax);
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
        synchronized (actionParams)
        {
            boolean onTarget = false;
            double currTime = TrcTimer.getCurrentTime();
            double deltaPos =
                actionParams.prevTime != null? actionParams.currStepRate*(currTime - actionParams.prevTime): 0.0;

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
                onTarget = true;
            }

            if (!onTarget)
            {
                tracer.traceDebug(instanceName, "deltaPos=" + deltaPos + ", actionParams=" + actionParams);
                performSetPosition(actionParams, false);
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
     * @param presets specifies an array of preset positions in scaled unit.
     */
    public void setPosPresets(double tolerance, double... presets)
    {
        posPresets = new TrcPresets(instanceName + ".posPresets", tolerance, presets);
    }   //setPosPresets

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
        if (posPresets != null && posPresets.validatePresetIndex(presetIndex))
        {
            setPosition(owner, delay, posPresets.getPresetValue(presetIndex), event, timeout);
        }
    }   //setPresetPosition

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
        if (posPresets != null)
        {
            double currValue = getPosition();
            int index = presetUp? posPresets.nextPresetIndexUp(currValue): posPresets.nextPresetIndexDown(currValue);

            if (index != -1)
            {
                setPresetPosition(owner, 0.0, index, null, 0.0);
            }
        }
    }   //setNextPresetPosition

    /**
     * This method sets the actuator to the next preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is completed, can be null if no ownership
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
