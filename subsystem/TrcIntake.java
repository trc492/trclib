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

package trclib.subsystem;

import java.util.concurrent.atomic.AtomicBoolean;

import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;
import trclib.sensor.TrcTrigger;
import trclib.sensor.TrcTrigger.TriggerMode;
import trclib.timer.TrcTimer;

/**
 * This class implements a generic platform independent intake subsystem. It contains a motor or a continuous
 * servo and optionally entry and exit sensors that detects if the intake has captured objects. It provides the
 * auto methods that allow the caller to call the intake subsystem to pickup or eject objects on a press of
 * a button and the intake subsystem will stop itself once it is done. While it provides the auto functionality
 * to pickup or eject objects, it also supports exclusive subsystem access by implementing TrcExclusiveSubsystem.
 * This enables the intake subsystem to be aware of multiple callers' access to the subsystem. While one caller starts
 * the intake for an operation, nobody can access it until the previous operation is done or canceled.
 */
public class TrcIntake implements TrcExclusiveSubsystem
{
    /**
     * This class contains all the parameters of the Intake Trigger.
     */
    public static class TriggerParams
    {
        private final TrcTrigger trigger;
        private final TrcEvent.Callback triggerCallback;
        private final Double analogTriggerThreshold;
        private final Boolean analogTriggerInverted;

        private TriggerMode triggerMode;
        private TrcEvent notifyEvent;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param trigger specifies the TrcTrigger object.
         * @param triggerCallback specifies the callback method to call when a trigger occurred, null if not provided.
         * @param threshold specifies the sensor's analog threshold value, null if trigger is digital.
         * @param triggerInverted specifies true if got object triggered below threshold, false if triggered above,
         *        null if trigger is digital.
         */
        public TriggerParams(
            TrcTrigger trigger, TrcEvent.Callback triggerCallback, Double threshold, Boolean triggerInverted)
        {
            this.trigger = trigger;
            this.triggerCallback = triggerCallback;
            this.analogTriggerThreshold = threshold;
            this.analogTriggerInverted = triggerInverted;
            this.triggerMode = TriggerMode.OnActive;
            this.notifyEvent = null;
        }   //Trigger

        /**
         * Constructor: Create an instance of the object.
         *
         * @param trigger specifies the TrcTrigger object.
         * @param triggerCallback specifies the callback method to call when a trigger occurred, null if not provided.
         */
        public TriggerParams(TrcTrigger trigger, TrcEvent.Callback triggerCallback)
        {
            this(trigger, triggerCallback, null, null);
        }   //Trigger

        /**
         * Constructor: Create an instance of the object.
         *
         * @param trigger specifies the TrcTrigger object.
         */
        public TriggerParams(TrcTrigger trigger)
        {
            this(trigger, null, null, null);
        }   //TriggerParams

        /**
         * This method returns the string form of all the parameters.
         *
         * @return string form of all the parameters.
         */
        @Override
        public String toString()
        {
            return "triggerCallback=" + (triggerCallback != null) +
                   ", threshold=" + analogTriggerThreshold +
                   ", inverted=" + analogTriggerInverted +
                   ", triggerMode=" + triggerMode +
                   ", event=" + notifyEvent;
        }   //toString

    }   //class TriggerParams

    /**
     * Specifies the operation types.
     */
    private enum Operation
    {
        IntakeForward,
        IntakeReverse,
        EjectForward,
        EjectReverse
    }   //enum Operation

    /**
     * This class encapsulates all the parameters required to perform the intake action.
     */
    private static class ActionParams
    {
        Operation operation;
        double intakePower;
        double retainPower;
        double finishDelay;
        TrcEvent event;
        double timeout;

        ActionParams(
            Operation operation, double intakePower, double retainPower, double finishDelay, TrcEvent event,
            double timeout)
        {
            this.operation = operation;
            this.intakePower = intakePower;
            this.retainPower = retainPower;
            this.finishDelay = finishDelay;
            this.event = event;
            this.timeout = timeout;
        }   //ActionParams

        @Override
        public String toString()
        {
            return "(op=" + operation +
                   ", intakePower=" + intakePower +
                   ", retainPower=" + retainPower +
                   ", finishDelay=" + finishDelay +
                   ", event=" + event +
                   ", timeout=" + timeout + ")";
        }   //toString

    }   //class ActionParams

    public final TrcDbgTrace tracer;
    private final String instanceName;
    public final TrcMotor motor;
    public final TriggerParams entryTrigger;
    public final TriggerParams exitTrigger;
    private final TrcTimer timer;
    private final TrcEvent timerEvent;
    private ActionParams actionParams = null;
    private String currOwner = null;
    private boolean delayFinishPending = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     * @param entryTrigger specifies the entry trigger parameters, can be null if no entry trigger.
     * @param exitTrigger specifies the exit trigger parameters, can be null if no exit trigger.
     */
    public TrcIntake(String instanceName, TrcMotor motor, TriggerParams entryTrigger, TriggerParams exitTrigger)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.motor = motor;
        this.entryTrigger = entryTrigger;
        this.exitTrigger = exitTrigger;

        timer = new TrcTimer(instanceName);
        timerEvent = new TrcEvent(instanceName + ".timerEvent");

        if (entryTrigger != null)
        {
            entryTrigger.trigger.enableTrigger(TriggerMode.OnBoth, this::entryTriggerCallback);
        }

        if (exitTrigger != null)
        {
            exitTrigger.trigger.enableTrigger(TriggerMode.OnBoth, this::exitTriggerCallback);
        }
    }   //TrcIntake

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     * @param entryTrigger specifies the entry trigger object, can be null if none.
     */
    public TrcIntake(String instanceName, TrcMotor motor, TriggerParams entryTrigger)
    {
        this(instanceName, motor, entryTrigger, null);
    }   //TrcIntake

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     */
    public TrcIntake(String instanceName, TrcMotor motor)
    {
        this(instanceName, motor, null, null);
    }   //TrcIntake

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName +
               ": pwr=" + getPower() +
               ", current=" + motor.getMotorCurrent() +
               ", isActive=" + isAutoActive() +
               ", hasObject=" + hasObject();
    }   //toString

    /**
     * This method returns the primary motor current.
     *
     * @return primary motor current.
     */
    public double getCurrent()
    {
        return motor.getCurrent();
    }   //getCurrent

    /**
     * This method returns the primary motor power.
     *
     * @return primary motor power.
     */
    public double getPower()
    {
        return motor.getPower();
    }   //getPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param delay specifies the delay in seconds to wait before setting the power of the motor.
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void setPower(String owner, double delay, double power, double duration, TrcEvent event)
    {
        motor.setPower(owner, delay, power, duration, event);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param delay specifies the delay in seconds to wait before setting the power of the motor.
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void setPower(double delay, double power, double duration, TrcEvent event)
    {
        setPower(null, delay, power, duration, event);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void setPower(double power, double duration, TrcEvent event)
    {
        setPower(null, 0.0, power, duration, event);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param delay specifies the delay in seconds to wait before setting the power of the motor.
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to have power set.
     */
    public void setPower(double delay, double power, double duration)
    {
        setPower(null, delay, power, duration, null);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to have power set.
     */
    public void setPower(double power, double duration)
    {
        setPower(null, 0.0, power, duration, null);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     */
    public void setPower(double power)
    {
        setPower(null, 0.0, power, 0.0, null);
    }   //setPower

    /**
     * This method is called to finish the operation and to clean up. It can be called either at the end of timeout
     * or when object has been captured or ejected to finish the operation and signal the caller for completion. It
     * can also be called if the caller explicitly cancel the operation in which case the event will be set to
     * canceled.
     *
     * @param completed specifies true if the operation is completed, false if canceled.
     */
    private void finish(boolean completed)
    {
        if (isAutoActive())
        {
            double power = completed && hasObject()? actionParams.retainPower: 0.0;

            tracer.traceDebug(
                instanceName,
                "completed=" + completed +
                ", timedOut=" + timerEvent.isSignaled() +
                ", hasObject=" + hasObject() +
                ", finishDelay=" + actionParams.finishDelay);
            tracer.traceInfo(
                instanceName, "Finishing intake: completed=" + completed +
                ", power=" + power + ", finishDelay=" + actionParams.finishDelay);
            setPower(actionParams.finishDelay, power, 0.0);
            delayFinishPending = actionParams.finishDelay > 0.0;
            timer.cancel();
            if (actionParams.event != null)
            {
                if (completed)
                {
                    actionParams.event.signal();
                }
                else
                {
                    actionParams.event.cancel();
                }
                actionParams.event = null;
            }

            actionParams = null;

            if (currOwner != null)
            {
                releaseExclusiveAccess(currOwner);
                currOwner = null;
            }
        }
        else
        {
            // TODO: Need to debug this, it doesn't work.
            tracer.traceInfo(
                instanceName, "Finishing inactive intake: completed=" + completed +
                ", delayFinishPending=" + delayFinishPending);
            if (!delayFinishPending)
            {
                motor.cancel();
            }
        }
    }   //finish

    /**
     * This method cancel a pending operation if any.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     */
    public void cancel(String owner)
    {
        tracer.traceInfo(instanceName, "owner=" + owner);
        if (validateOwnership(owner))
        {
            finish(false);
        }
    }   //cancel

    /**
     * This method cancel a pending operation if any.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method performs the action.
     *
     * @param context specifies the action parameters.
     */
    private void performAction(Object context)
    {
        ActionParams actionParams = (ActionParams) context;
        boolean objCaptured = hasObject();
        boolean doingIntake =
            actionParams.operation == Operation.IntakeForward || actionParams.operation == Operation.IntakeReverse;

        tracer.traceInfo(instanceName, "Action: " + actionParams + ", hasObject=" + objCaptured);
        if (doingIntake ^ objCaptured)
        {
            // Picking up object and we don't have one yet, or ejecting object and we still have one.
            motor.setPower(actionParams.intakePower);
            if (actionParams.timeout > 0.0)
            {
                timerEvent.setCallback(this::actionTimedOut, null);
                timer.set(actionParams.timeout, timerEvent);
            }
        }
        else
        {
            // Picking up object but we already have one, or ejecting object but there isn't any.
            finish(true);
        }
    }   //performAction

    /**
     * This method is called when the action has timed out.
     *
     * @param context not used.
     */
    private void actionTimedOut(Object context)
    {
        tracer.traceDebug(instanceName, "Timed out: actionParams=" + actionParams);
        finish(false);
    }   //actionTimedOut

    /**
     * This method is called when the entry trigger state has changed.
     *
     * @param context specifies the trigger state.
     */
    private void entryTriggerCallback(Object context)
    {
        boolean active = ((AtomicBoolean) context).get();

        tracer.traceInfo(instanceName, "active=" + active + ", actionParams=" + actionParams);
        if (actionParams != null)
        {
            // Only need to finish if object is reverse ejecting or reverse intaking.
            // Or if we only have an entry trigger and we are forward ejecting or forward intaking.
            if (actionParams.operation == Operation.EjectReverse && !active ||
                actionParams.operation == Operation.IntakeReverse && active ||
                exitTrigger == null &&
                (actionParams.operation == Operation.EjectForward && !active ||
                 actionParams.operation == Operation.IntakeForward && active))
            {
                // The object has been ejected.
                finish(true);
            }
        }

        if (entryTrigger.notifyEvent != null &&
            (entryTrigger.triggerMode == TriggerMode.OnBoth ||
             entryTrigger.triggerMode == TriggerMode.OnActive && active ||
             entryTrigger.triggerMode == TriggerMode.OnInactive && !active))
        {
            entryTrigger.notifyEvent.signal();
        }

        if (entryTrigger.triggerCallback != null)
        {
            entryTrigger.triggerCallback.notify(context);
        }
    }   //entryTriggerCallback

    /**
     * This method is called when the exit trigger state has changed.
     *
     * @param context specifies the trigger state.
     */
    private void exitTriggerCallback(Object context)
    {
        boolean active = ((AtomicBoolean) context).get();

        tracer.traceInfo(instanceName, "active=" + active + ", actionParams=" + actionParams);
        if (actionParams != null)
        {
            // Only need to finish if object is forward ejecting or forward intaking.
            if (actionParams.operation == Operation.EjectForward && !active ||
                actionParams.operation == Operation.IntakeForward && active)
            {
                // We have either captured an object or ejected it.
                finish(true);
            }
        }

        if (exitTrigger.notifyEvent != null &&
            (exitTrigger.triggerMode == TriggerMode.OnBoth ||
             exitTrigger.triggerMode == TriggerMode.OnActive && active ||
             exitTrigger.triggerMode == TriggerMode.OnInactive && !active))
        {
            exitTrigger.notifyEvent.signal();
        }

        if (exitTrigger.triggerCallback != null)
        {
            exitTrigger.triggerCallback.notify(context);
        }
    }   //exitTriggerCallback

    /**
     * This method registers a notification event to be signaled when the entry trigger occurred.
     *
     * @param triggerMode specifies trigger mode that will signal the event.
     * @param notifyEvent specifies the event to signal when entry trigger occurred.
     * @return true if success, false if there is no entry trigger or it is already enabled by someone else.
     */
    public boolean registerEntryTriggerNotifyEvent(TriggerMode triggerMode, TrcEvent notifyEvent)
    {
        boolean success = false;

        if (entryTrigger != null && entryTrigger.notifyEvent == null)
        {
            entryTrigger.triggerMode = triggerMode;
            entryTrigger.notifyEvent = notifyEvent;
            success = true;
        }

        return success;
    }   //registerEntryTriggerNotifyCallback

    /**
     * This method unregisters the notification event for the entry trigger.
     *
     * @return true if success, false if trigger does not exist or has no notification event registered.
     */
    public boolean unregisterEntryTriggerNotifyEvent()
    {
        boolean success = false;

        if (entryTrigger != null && entryTrigger.notifyEvent != null)
        {
            // Must disable notifyEvent before disabling trigger.
            entryTrigger.notifyEvent = null;
            success = true;
        }

        return success;
    }   //unregisterEntryTriggerNotifyCallback

    /**
     * This method registers a notification event to be signaled when the exit trigger occurred.
     *
     * @param triggerMode specifies trigger mode that will signal the event.
     * @param notifyEvent specifies the event to signal when exit trigger occurred.
     * @return true if success, false if there is no exit trigger or it is already enabled by someone else.
     */
    public boolean registerExitTriggerNotifyEvent(TriggerMode triggerMode, TrcEvent notifyEvent)
    {
        boolean success = false;

        if (exitTrigger != null && exitTrigger.notifyEvent == null)
        {
            exitTrigger.triggerMode = triggerMode;
            exitTrigger.notifyEvent = notifyEvent;
            success = true;
        }

        return success;
    }   //registerExitTriggerNotifyCallback

    /**
     * This method unregisters the notification event for the exit trigger.
     *
     * @return true if success, false if trigger does not exist or has no notification event registered.
     */
    public boolean unregisterExitTriggerNotifyEvent()
    {
        boolean success = false;

        if (exitTrigger != null && exitTrigger.notifyEvent != null)
        {
            // Must disable notifyEvent before disabling trigger.
            exitTrigger.notifyEvent = null;
            success = true;
        }

        return success;
    }   //unregisterExitTriggerNotifyCallback

    /**
     * This method performs an auto operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once object is picked up or ejected in the intake at which time the given event will
     * be signaled.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param operation specifies the intake operation.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured, applicable only to intake
     *        object.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    private void autoOperation(
        String owner, Operation operation, double delay, double power, double retainPower, double finishDelay,
        TrcEvent event, double timeout)
    {
        if (power == 0.0)
        {
            throw new RuntimeException("Must provide non-zero power to perform Auto Operation.");
        }
        else if (entryTrigger == null)
        {
            throw new RuntimeException("Must have entry sensor to perform Auto Operation.");
        }

        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, event, tracer);
        if (releaseOwnershipEvent != null) event = releaseOwnershipEvent;
        //
        // Make sure the caller has ownership.
        //
        if (validateOwnership(owner))
        {
            power = Math.abs(power);
            if (operation == Operation.EjectReverse || operation == Operation.IntakeReverse) power = -power;
            actionParams = new ActionParams(operation, power, retainPower, finishDelay, event, timeout);
            if (delay > 0.0)
            {
                timerEvent.setCallback(this::performAction, actionParams);
                timer.set(delay, timerEvent);
            }
            else
            {
                performAction(actionParams);
            }
        }
    }   //autoOperation

    /**
     * This method performs the auto intake forward operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is picked up in the intake at which time the given event
     * will be signaled.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoIntakeForward(
        String owner, double delay, double power, double retainPower, double finishDelay,
        TrcEvent event, double timeout)
    {
        autoOperation(owner, Operation.IntakeForward, delay, power, retainPower, finishDelay, event, timeout);
    }   //autoIntakeForward

    /**
     * This method performs the auto intake forward operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is picked up in the intake at which time the given event
     * will be signaled.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoIntakeForward(
        double delay, double power, double retainPower, double finishDelay, TrcEvent event, double timeout)
    {
        autoOperation(null, Operation.IntakeForward, delay, power, retainPower, finishDelay, event, timeout);
    }   //autoIntakeForward

    /**
     * This method performs the auto intake forward operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is picked up in the intake.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     */
    public void autoIntakeForward(double delay, double power, double retainPower, double finishDelay)
    {
        autoOperation(null, Operation.IntakeForward, delay, power, retainPower, finishDelay, null, 0.0);
    }   //autoIntakeForward

    /**
     * This method performs the auto intake forward operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is picked up in the intake.
     *
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     */
    public void autoIntakeForward(double power, double retainPower, double finishDelay)
    {
        autoOperation(null, Operation.IntakeForward, 0.0, power, retainPower, finishDelay, null, 0.0);
    }   //autoIntakeForward

    /**
     * This method performs the auto intake reverse operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is picked up in the intake at which time the given event
     * will be signaled.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoIntakeReverse(
        String owner, double delay, double power, double retainPower, double finishDelay,
        TrcEvent event, double timeout)
    {
        autoOperation(owner, Operation.IntakeReverse, delay, power, retainPower, finishDelay, event, timeout);
    }   //autoIntakeReverse

    /**
     * This method performs the auto intake reverse operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is picked up in the intake at which time the given event
     * will be signaled.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoIntakeReverse(
        double delay, double power, double retainPower, double finishDelay, TrcEvent event, double timeout)
    {
        autoOperation(null, Operation.IntakeReverse, delay, power, retainPower, finishDelay, event, timeout);
    }   //autoIntakeReverse

    /**
     * This method performs the auto intake reverse operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is picked up in the intake.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     */
    public void autoIntakeReverse(double delay, double power, double retainPower, double finishDelay)
    {
        autoOperation(null, Operation.IntakeReverse, delay, power, retainPower, finishDelay, null, 0.0);
    }   //autoIntakeReverse

    /**
     * This method performs the auto intake reverse operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is picked up in the intake.
     *
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     */
    public void autoIntakeReverse(double power, double retainPower, double finishDelay)
    {
        autoOperation(null, Operation.IntakeReverse, 0.0, power, retainPower, finishDelay, null, 0.0);
    }   //autoIntakeReverse

    /**
     * This method performs the auto eject forward operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is ejected in the intake at which time the given event
     * will be signaled.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to eject.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoEjectForward(
        String owner, double delay, double power, double finishDelay, TrcEvent event, double timeout)
    {
        autoOperation(owner, Operation.EjectForward, delay, power, 0.0, finishDelay, event, timeout);
    }   //autoEjectForward

    /**
     * This method performs the auto eject forward operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is ejected in the intake at which time the given event
     * will be signaled.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to eject.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoEjectForward(double delay, double power, double finishDelay, TrcEvent event, double timeout)
    {
        autoOperation(null, Operation.EjectForward, delay, power, 0.0, finishDelay, event, timeout);
    }   //autoEjectForward

    /**
     * This method performs the auto eject forward operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is ejected in the intake.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to eject.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     */
    public void autoEjectForward(double delay, double power, double finishDelay)
    {
        autoOperation(null, Operation.EjectForward, delay, power, 0.0, finishDelay, null, 0.0);
    }   //autoEjectForward

    /**
     * This method performs the auto eject forward operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is ejected in the intake.
     *
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to eject.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     */
    public void autoEjectForward(double power, double finishDelay)
    {
        autoOperation(null, Operation.EjectForward, 0.0, power, 0.0, finishDelay, null, 0.0);
    }   //autoEjectForward

    /**
     * This method performs the auto eject reverse operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is ejected in the intake at which time the given event
     * will be signaled.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to eject.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoEjectReverse(
        String owner, double delay, double power, double finishDelay, TrcEvent event, double timeout)
    {
        autoOperation(owner, Operation.EjectReverse, delay, power, 0.0, finishDelay, event, timeout);
    }   //autoEjectReverse

    /**
     * This method performs the auto eject reverse operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is ejected in the intake at which time the given event
     * will be signaled.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to eject.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoEjectReverse(double delay, double power, double finishDelay, TrcEvent event, double timeout)
    {
        autoOperation(null, Operation.EjectReverse, delay, power, 0.0, finishDelay, event, timeout);
    }   //autoEjectReverse

    /**
     * This method performs the auto eject reverse operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is ejected in the intake.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to eject.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     */
    public void autoEjectReverse(double delay, double power, double finishDelay)
    {
        autoOperation(null, Operation.EjectReverse, delay, power, 0.0, finishDelay, null, 0.0);
    }   //autoEjectReverse

    /**
     * This method performs the auto eject reverse operation. It allows the caller to start the intake spinning at
     * the given power and it will stop itself once object is ejected in the intake.
     *
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to eject.
     * @param finishDelay specifies the delay in seconds to fnish the auto operation to give it extra time spinning
     *        the intake.
     */
    public void autoEjectReverse(double power, double finishDelay)
    {
        autoOperation(null, Operation.EjectReverse, 0.0, power, 0.0, finishDelay, null, 0.0);
    }   //autoEjectReverse

    /**
     * This method returns the sensor value read from the analog sensor of the trigger.
     *
     * @param trigger specifies the trigger.
     * @return analog trigger sensor value.
     */
    public double getSensorValue(TriggerParams trigger)
    {
        return trigger != null && trigger.analogTriggerThreshold != null? trigger.trigger.getSensorValue(): 0.0;
    }   //getSensorValue

    /**
     * This method returns the sensor state read from the digital sensor of the trigger.
     *
     * @return digital trigger sensor state.
     */
    public boolean getSensorState(TriggerParams trigger)
    {
        return trigger != null && trigger.analogTriggerThreshold == null && trigger.trigger.getSensorState();
    }   //getSensorState

    /**
     * This method checks if the trigger sensor has detected an object.
     *
     * @param trigger specifies the trigger to check.
     * @return true if trigger sensor detected an object, false otherwise.
     */
    public boolean isTriggerActive(TriggerParams trigger)
    {
        boolean active = false;

        if (trigger != null)
        {
            if (trigger.analogTriggerThreshold != null)
            {
                active = getSensorValue(trigger) > trigger.analogTriggerThreshold;
                if (trigger.analogTriggerInverted)
                {
                    active = !active;
                }
            }
            else
            {
                active = getSensorState(trigger);
            }
        }

        return active;
    }   //isTriggerActive

    /**
     *
     * This method checks if object is detected in the intake.
     *
     * @return true if object is detected in the intake, false otherwise.
     */
    public boolean hasObject()
    {
        return isTriggerActive(entryTrigger) || isTriggerActive(exitTrigger);
    }   //hasObject

    /**
     * This method checks if auto operation is active.
     *
     * @return true if auto operation is in progress, false otherwise.
     */
    public boolean isAutoActive()
    {
        return actionParams != null;
    }   //isAutoActive

}   //class TrcIntake
