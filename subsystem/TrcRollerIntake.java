/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements a platform independent auto-assist roller intake subsystem. It contains one or two motors
 * and optionally one or two sensors that detects the position of the object. It provides the autoAssist methods
 * that allow the caller to intake or eject objects on a press of a button and the intake subsystem will automatically
 * grab the object once it is within grasp. While it provides the auto-assist functionality to intake or eject
 * objects, it also supports exclusive subsystem access by implementing TrcExclusiveSubsystem. This enables the
 * intake subsystem to be aware of multiple callers' access to the subsystem. While one caller starts the subsystem
 * for an operation, nobody can access it until the previous caller is done with the operation.
 */
public class TrcRollerIntake implements TrcExclusiveSubsystem
{
    /**
     * This class contains Intake parameters.
     */
    public static class IntakeParams
    {
        private double intakePower = 1.0;
        private double ejectPower = 1.0;
        private double retainPower = 0.0;
        private double intakeFinishDelay = 0.0;
        private double ejectFinishDelay = 0.0;

        /**
         * This method returns the string format of the Intake parameters.
         *
         * @return string format of the parameters.
         */
        @Override
        public String toString()
        {
            return "(intakePower=" + intakePower +
                   ",ejectPower=" + ejectPower +
                   ",retainPower=" + retainPower +
                   ",intakeFinishDelay=" + intakeFinishDelay +
                   ",ejectFinishDelay=" + ejectFinishDelay + ")";
        }   //toString

        /**
         * This method sets various power levels of the Roller Intake.
         *
         * @param intakePower specifies the intake power.
         * @param ejectPower specifies the eject power.
         * @param retainPower specifies the retain power.
         * @return this parameter object.
         */
        public IntakeParams setPowerLevels(double intakePower, double ejectPower, double retainPower)
        {
            this.intakePower = intakePower;
            this.ejectPower = ejectPower;
            this.retainPower = retainPower;
            return this;
        }   //setPowerLevels

        /**
         * This method sets various finish delays of the Roller Intake.
         *
         * @param intakeFinishDelay specifies the intake finish delay in seconds.
         * @param ejectFinishDelay specifies the eject finish delay in seconds.
         * @return this parameter object.
         */
        public IntakeParams setFinishDelays(double intakeFinishDelay, double ejectFinishDelay)
        {
            this.intakeFinishDelay = intakeFinishDelay;
            this.ejectFinishDelay = ejectFinishDelay;
            return this;
        }   //setFinishDelays

    }   //class IntakeParams

    public enum TriggerAction
    {
        NoAction,       // Do nothing when trigger occurs.
        StartOnTrigger, // Start roller intake when trigger occurs.
        FinishOnTrigger // Finish roller intake operation when trigger occurs.
    }   //enum TriggerAction

    /**
     * This class contains all the parameters of the Intake Trigger. The parameters specify the action it will take
     * when the trigger occurs. The trigger can optionally provide a notification callback.
     */
    public static class TriggerParams
    {
        private final TrcTrigger trigger;
        private final TriggerAction triggerAction;
        private final TriggerMode triggerMode;
        private final TrcEvent.Callback triggerCallback;
        private final Object callbackContext;

        public TriggerParams(
            TrcTrigger trigger, TriggerAction triggerAction, TriggerMode triggerMode, TrcEvent.Callback callback,
            Object callbackContext)
        {
            this.trigger = trigger;
            this.triggerAction = triggerAction;
            this.triggerMode = triggerMode;
            this.triggerCallback = callback;
            this.callbackContext = callbackContext;
        }   //TriggerParams

        /**
         * This method returns the string form of all the parameters.
         *
         * @return string form of all the parameters.
         */
        @Override
        public String toString()
        {
            return "trigger=" + trigger +
                   ", triggerAction=" + triggerAction +
                   ", triggerMode=" + triggerMode +
                   ", triggerCallback=" + (triggerCallback != null) +
                   ", callbackContext=" + (callbackContext != null);
        }   //toString

    }   //class TriggerParams

    /**
     * This class encapsulates all the parameters required to perform the action.
     */
    private static class ActionParams
    {
        boolean intakeAction;
        String owner;
        TrcEvent completionEvent;
        double timeout;

        ActionParams(
            boolean intakeAction, String owner, TrcEvent completionEvent, double timeout)
        {
            this.intakeAction = intakeAction;
            this.owner = owner;
            this.completionEvent = completionEvent;
            this.timeout = timeout;
        }   //ActionParams

        @Override
        public String toString()
        {
            return "(intakeAction=" + intakeAction +
                   ",owner=" + owner +
                   ",completionEvent=" + completionEvent +
                   ",timeout=" + timeout + ")";
        }   //toString

    }   //class ActionParams

    public final TrcDbgTrace tracer;
    private final String instanceName;
    public final TrcMotor motor;
    private final IntakeParams intakeParams;
    private final TriggerParams frontTriggerParams;
    private final TriggerParams backTriggerParams;
    private final TrcTimer timer;
    private ActionParams actionParams = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor object.
     * @param intakeParams specifies the intake parameters.
     * @param frontTriggerParams specifies the front sensor trigger parameters, null if no front sensor.
     * @param backTriggerParams specifies the back sensor trigger parameters, null if no back sensor.
     */
    public TrcRollerIntake(
        String instanceName, TrcMotor motor, IntakeParams intakeParams, TriggerParams frontTriggerParams,
        TriggerParams backTriggerParams)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.motor = motor;
        this.intakeParams = intakeParams;
        this.frontTriggerParams = frontTriggerParams;
        this.backTriggerParams = backTriggerParams;
        timer = new TrcTimer(instanceName);
    }   //TrcRollerIntake

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
     * This method processes a trigger event.
     *
     * @param active specifies the trigger state.
     * @param triggerParams specifies the trigger parameters.
     * @param canceled specifies true if trigger was disabled (doesn't really happen, handle it regardless).
     */
    private void processTrigger(boolean active, TriggerParams triggerParams, boolean canceled)
    {
        tracer.traceDebug(
            instanceName, "Trigger: active=%s, triggerParams=%s, actionParams=%s, canceled=%s",
            active, triggerParams, actionParams, canceled);

        if (actionParams != null)
        {
            if (!canceled && active)
            {
                if (triggerParams == frontTriggerParams && triggerParams.triggerAction == TriggerAction.StartOnTrigger)
                {
                    // Start the Intake Motor and disable front trigger.
                    motor.setPower(actionParams.owner, 0.0, intakeParams.intakePower, 0.0, null);
                    setFrontTriggerEnabled(false);
                }
                else if (triggerParams == backTriggerParams &&
                         triggerParams.triggerAction == TriggerAction.FinishOnTrigger)
                {
                    // Finish the operation and disable the back trigger.
                    setBackTriggerEnabled(false);
                    double finishDelay =
                        actionParams.intakeAction? intakeParams.intakeFinishDelay: intakeParams.ejectFinishDelay;
                    if (finishDelay > 0.0)
                    {
                        timer.set(finishDelay, (ctxt, cancel)-> finishAction(!cancel));
                    }
                    else
                    {
                        finishAction(true);
                    }
                }
            }

            if (triggerParams.triggerCallback != null &&
                (canceled ||
                 triggerParams.triggerMode == TriggerMode.OnBoth ||
                 triggerParams.triggerMode == TriggerMode.OnActive && active ||
                 triggerParams.triggerMode == TriggerMode.OnInactive && !active))
            {
                // Caller has registered a callback with a specified trigger mode, call it.
                triggerParams.triggerCallback.notify(triggerParams.callbackContext, canceled);
            }
        }
    }   //processTrigger

    /**
     * This method enables/disables the front trigger of the Intake.
     *
     * @param enabled specifies true to enable the trigger, false to disable.
     */
    private void setFrontTriggerEnabled(boolean enabled)
    {
        if (frontTriggerParams != null)
        {
            if (enabled)
            {
                frontTriggerParams.trigger.enableTrigger(
                    TriggerMode.OnBoth,
                    (context, canceled)->processTrigger(
                        ((AtomicBoolean) context).get(), frontTriggerParams, canceled));
            }
            else
            {
                frontTriggerParams.trigger.disableTrigger();
            }
        }
    }   //setFrontTriggerEnabled

    /**
     * This method enables/disables the back trigger of the Intake.
     *
     * @param enabled specifies true to enable the trigger, false to disable.
     */
    private void setBackTriggerEnabled(boolean enabled)
    {
        if (backTriggerParams != null)
        {
            if (enabled)
            {
                backTriggerParams.trigger.enableTrigger(
                    TriggerMode.OnBoth,
                    (context, canceled)->processTrigger(
                        ((AtomicBoolean) context).get(), backTriggerParams, canceled));
            }
            else
            {
                backTriggerParams.trigger.disableTrigger();
            }
        }
    }   //setBackTriggerEnabled

    /**
     * This method checks if auto operation is active.
     *
     * @return true if auto operation is in progress, false otherwise.
     */
    public boolean isAutoActive()
    {
        return actionParams != null;
    }   //isAutoActive

    /**
     * This method returns the front trigger state.
     *
     * @return front trigger state, false if there is no front trigger.
     */
    public boolean getFrontTriggerState()
    {
        return frontTriggerParams != null && frontTriggerParams.trigger.getTriggerState();
    }   //getFrontTriggerState

    /**
     * This method returns the back trigger state.
     *
     * @return back trigger state, false if there is no back trigger.
     */
    public boolean getBackTriggerState()
    {
        return backTriggerParams != null && backTriggerParams.trigger.getTriggerState();
    }   //getBackTriggerState

    /**
     *
     * This method checks if object is detected.
     *
     * @return true if object is detected, false otherwise.
     */
    public boolean hasObject()
    {
        return getFrontTriggerState() || getBackTriggerState();
    }   //hasObject

    /**
     * This method returns the sensor value read from the front analog sensor.
     *
     * @return front analog sensor value, or zero if there is no front sensor.
     */
    public double getFrontSensorValue()
    {
        return frontTriggerParams != null? frontTriggerParams.trigger.getSensorValue(): 0.0;
    }   //getFrontSensorValue

    /**
     * This method returns the sensor value read from the back analog sensor.
     *
     * @return back analog sensor value, or zero if there is no back sensor.
     */
    public double getBackSensorValue()
    {
        return backTriggerParams != null? backTriggerParams.trigger.getSensorValue(): 0.0;
    }   //getBackSensorValue

    /**
     * This method checks if the intake motor is ON.
     *
     * @return true if intake motor is ON, false if open.
     */
    public boolean isActive()
    {
        return motor.getPower() != 0.0;
    }   //isActive

    /**
     * This method returns the intake motor current.
     *
     * @return intake motor current.
     */
    public double getCurrent()
    {
        return motor.getCurrent();
    }   //getCurrent

    /**
     * This method returns the current intake motor power.
     *
     * @return current intake motor power.
     */
    public double getPower()
    {
        return motor.getPower();
    }   //getPower

    //
    // Primitive actions.
    //

    /**
     * This method spins the intake motor with the specified power and optional duration.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay.
     * @param power specifies the percentage power (range -1.0 to 1.0).
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void setPower(String owner, double delay, double power, double duration, TrcEvent event)
    {
        tracer.traceDebug(
            instanceName, "owner=%s,delay=%.3f,power=%.3f,duration=%.3f,event=%s",
            owner, delay, power, duration, event);
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, event, tracer);
        if (releaseOwnershipEvent != null) event = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            // Cancel previous auto operation if there is one.
            finishAction(false);
            if (event != null)
            {
                event.clear();
            }
            motor.setPower(owner, delay, power, duration, event);
        }
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
     * This method stops the intake motor.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     */
    public void stop(String owner)
    {
        setPower(owner, 0.0, 0.0, 0.0, null);
    }   //stop

    /**
     * This method stops the intake motor.
     */
    public void stop()
    {
        setPower(null, 0.0, 0.0, 0.0, null);
    }   //stop

    //
    // Manual actions.
    //

    /**
     * This method starts the motor with intake power and optionally specifies the duration after which to turn off
     * the motor automatically and signal an event if provided.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay.
     * @param power specifies the intake power, null if use default power.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void intake(String owner, double delay, Double power, double duration, TrcEvent event)
    {
        setPower(owner, delay, power != null? power: intakeParams.intakePower, duration, event);
    }   //intake

    /**
     * This method starts the motor with intake power and optionally specifies the duration after which to turn off
     * the motor automatically and signal an event if provided.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param power specifies the intake power, null if use default power.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void intake(String owner, Double power, double duration, TrcEvent event)
    {
        intake(owner, 0.0, power, duration, event);
    }   //intake

    /**
     * This method starts the motor with intake power and optionally specifies the duration after which to turn off
     * the motor automatically and signal an event if provided.
     *
     * @param power specifies the intake power, null if use default power.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void intake(Double power, double duration, TrcEvent event)
    {
        intake(null, 0.0, power, duration, event);
    }   //intake

    /**
     * This method starts the motor with intake power.
     *
     * @param power specifies the intake power, null if use default power.
     */
    public void intake(Double power)
    {
        intake(null, 0.0, power, 0.0, null);
    }   //intake

    /**
     * This method starts the motor with intake power.
     */
    public void intake()
    {
        intake(null, 0.0, intakeParams.intakePower, 0.0, null);
    }   //intake

    /**
     * This method starts the motor with eject power and optionally specifies the duration after which to turn off
     * the motor automatically and signal an event if provided.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay.
     * @param power specifies the intake power, null if use default power.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void eject(String owner, double delay, Double power, double duration, TrcEvent event)
    {
        setPower(owner, delay, power != null? power: intakeParams.ejectPower, duration, event);
    }   //eject

    /**
     * This method starts the motor with eject power and optionally specifies the duration after which to turn off
     * the motor automatically and signal an event if provided.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param power specifies the intake power, null if use default power.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void eject(String owner, Double power, double duration, TrcEvent event)
    {
        eject(owner, 0.0, power, duration, event);
    }   //eject

    /**
     * This method starts the motor with eject power and optionally specifies the duration after which to turn off
     * the motor automatically and signal an event if provided.
     *
     * @param power specifies the intake power, null if use default power.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void eject(Double power, double duration, TrcEvent event)
    {
        eject(null, 0.0, power, duration, event);
    }   //eject

    /**
     * This method starts the motor with eject power.
     *
     * @param power specifies the intake power, null if use default power.
     */
    public void eject(Double power)
    {
        eject(null, 0.0, power, 0.0, null);
    }   //eject

    /**
     * This method starts the motor with eject power.
     */
    public void eject()
    {
        eject(null, 0.0, intakeParams.intakePower, 0.0, null);
    }   //eject

    //
    // Auto actions.
    //

    /**
     * This method is called to finish and clean up the auto action.
     *
     * @param completed specifies true to complete the action, false to cancel.
     */
    private void finishAction(boolean completed)
    {
        // Do clean up only if auto action is enabled.
        if (actionParams != null)
        {
            boolean gotObject = hasObject();

            tracer.traceInfo(
                instanceName, "FinishAction(completed=%s): isActive=%s, hasObject=%s, actionParams=%s",
                completed, isActive(), gotObject, actionParams);
            timer.cancel();
            if (completed)
            {
                double power = actionParams.intakeAction && gotObject? intakeParams.retainPower: 0.0;
                motor.setPower(actionParams.owner, 0.0, power, 0.0, null);
            }
            else
            {
                // Operation was canceled, cancel operation to release ownership if any.
                motor.cancel();
            }

            if (actionParams.completionEvent != null)
            {
                if (completed)
                {
                    actionParams.completionEvent.signal();
                }
                else
                {
                    actionParams.completionEvent.cancel();
                }
                actionParams.completionEvent = null;
            }
            actionParams = null;
        }
        else if (!completed)
        {
            // We are canceling but there was no pending auto action, just stop the motor.
            motor.cancel();
        }
    }   //finishAction

    /**
     * This method cancels the auto-assist operation and to clean up. It is called by enableAutoAssist to cancel a
     * previous operation or if the auto-assist has set a timeout and it has expired. Auto-assist will not be canceled
     * even if the sensor trigger caused it to grab an object. If a timeout is not set, auto-assist remains enabled
     * and can auto grab an object over and over again until the user calls this method to cancel the operation.
     *
     * @param context not used.
     * @param canceled not used.
     */
    private void actionTimedOut(Object context, boolean canceled)
    {
        tracer.traceDebug(instanceName, "Auto action timed out.");
        finishAction(false);
    }   //actionTimedOut

    /**
     * This method performs the auto action which is to spin the motor for intake if we don't have the object or eject
     * if we still have the object. It arms the sensor trigger to detect object possession. If there is a timeout, it
     * arms the timeout timer for canceling the auto action when the timer expires.
     *
     * @param context specifies the action parameters.
     * @param canceled specifies true if canceled.
     */
    private void performAction(Object context, boolean canceled)
    {
        if (!canceled)
        {
            ActionParams ap = (ActionParams) context;
            boolean gotObject = hasObject();
            boolean actionPending = false;
    
            if (ap.intakeAction && !gotObject)
            {
                // We are intaking but we don't have the object yet, enable front and back trigger if there is one.
                tracer.traceDebug(instanceName, "Start Intake.");
                setBackTriggerEnabled(true);
                if (frontTriggerParams != null && frontTriggerParams.triggerAction == TriggerAction.StartOnTrigger)
                {
                    setFrontTriggerEnabled(true);
                }
                else
                {
                    motor.setPower(ap.owner, 0.0, intakeParams.intakePower, 0.0, null);
                }
                actionPending = true;
            }
            else if (!ap.intakeAction && gotObject)
            {
                // We are ejecting and we still have the object.
                tracer.traceDebug(instanceName, "Start Eject.");
                motor.setPower(ap.owner, 0.0, intakeParams.ejectPower, 0.0, null);
                actionPending = true;
            }

            if (actionPending)
            {
                if (ap.timeout > 0.0)
                {
                    // Set a timeout and cancel auto-assist if timeout has expired.
                    tracer.traceDebug(instanceName, "Set timeout " + ap.timeout + " sec");
                    timer.set(ap.timeout, this::actionTimedOut, null);
                }
            }
            else
            {
                // Action already completed, we are done.
                finishAction(true);
            }
        }
    }   //performAction

    /**
     * This method enables auto operation. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param intakeAction specifies true if the action is intake, false if eject.
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param completionEvent specifies the event to signal when the action is completed.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     */
    private void autoAction(boolean intakeAction, String owner, double delay, TrcEvent completionEvent, double timeout)
    {
        if ((frontTriggerParams == null || frontTriggerParams.triggerAction == TriggerAction.NoAction) &&
            (backTriggerParams == null || backTriggerParams.triggerAction == TriggerAction.NoAction))
        {
            throw new RuntimeException("Must have sensor to perform Auto Operation.");
        }
        // This is an auto operation, make sure the caller has ownership.
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            // In case there is an existing auto operation still pending, cancel it first.
            finishAction(false);
            actionParams = new ActionParams(intakeAction, owner, completionEvent, timeout);
            if (delay > 0.0)
            {
                timer.set(delay, this::performAction, actionParams);
            }
            else
            {
                performAction(actionParams, false);
            }
        }
    }   //autoAction

    /**
     * This method enables auto intake. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param completionEvent specifies the event to signal when the action is completed.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     */
    public void autoIntake(String owner, double delay, TrcEvent completionEvent, double timeout)
    {
        autoAction(true, owner, delay, completionEvent, timeout);
    }   //autoIntake

    /**
     * This method enables auto intake. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param completionEvent specifies the event to signal when the action is completed.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     */
    public void autoIntake(String owner, TrcEvent completionEvent, double timeout)
    {
        autoAction(true, owner, 0.0, completionEvent, timeout);
    }   //autoIntake

    /**
     * This method enables auto intake. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     */
    public void autoIntake(String owner)
    {
        autoAction(true, owner, 0.0, null, 0.0);
    }   //autoIntake

    /**
     * This method enables auto eject. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param completionEvent specifies the event to signal when the action is completed.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     */
    public void autoEject(String owner, double delay, TrcEvent completionEvent, double timeout)
    {
        autoAction(false, owner, delay, completionEvent, timeout);
    }   //autoEject

    /**
     * This method enables auto eject. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param completionEvent specifies the event to signal when the action is completed.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     */
    public void autoEject(String owner, TrcEvent completionEvent, double timeout)
    {
        autoAction(false, owner, 0.0, completionEvent, timeout);
    }   //autoEject

    /**
     * This method enables auto eject. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     */
    public void autoEject(String owner)
    {
        autoAction(false, owner, 0.0, null, 0.0);
    }   //autoEject

    /**
     * This method cancels the auto-assist operation and to clean up. It is called by the user for canceling the
     * operation.
     */
    public void cancel()
    {
        finishAction(false);
    }   //cancel

}   //class TrcRollerIntake
