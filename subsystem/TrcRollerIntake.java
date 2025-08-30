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
        private TrcEvent.Callback triggerCallback;
        private Object callbackContext;

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
    * This class contains all the parameters for the Roller Intake.
    */
    public static class Params
    {
        private TrcMotor motor = null;
        private TriggerParams frontTriggerParams = null;
        private TriggerParams backTriggerParams = null;
        private double intakePower = 0.0;
        private double ejectPower = 0.0;
        private double retainPower = 0.0;
        private double intakeFinishDelay = 0.0;
        private double ejectFinishDelay = 0.0;
 
        /**
         * This method returns the string form of all the parameters.
         *
         * @return string form of all the parameters.
         */
        @Override
        public String toString()
        {
            return "motor=" + motor +
                   ",frontTriggerParams=" + frontTriggerParams +
                   ",backTriggerParams=" + backTriggerParams +
                   ",intakePower=" + intakePower +
                   ",ejectPower=" + ejectPower +
                   ",retainPower=" + retainPower +
                   ",intakeFinishDelay=" + intakeFinishDelay +
                   ",ejectFinishDelay=" + ejectFinishDelay;
        }   //toString
 
        /**
         * This method sets the motor object for the grabber.
         *
         * @param motor specifies the motor object.
         * @return this parameter object.
         */
        public Params setMotor(TrcMotor motor)
        {
            this.motor = motor;
            return this;
        }   //setMotor
 
        /**
         * This method sets the front trigger and its parameters.
         *
         * @param triggerParams specifies the trigger parameters.
         * @return this parameter object.
         */
        public Params setFrontTrigger(TriggerParams triggerParams)
        {
            this.frontTriggerParams = triggerParams;
            return this;
        }   //setFrontTrigger

        /**
         * This method sets the back trigger and its parameters.
         *
         * @param triggerParams specifies the trigger parameters.
         * @return this parameter object.
         */
        public Params setBackTrigger(TriggerParams triggerParams)
        {
            this.backTriggerParams = triggerParams;
            return this;
        }   //setBackTrigger

        /**
         * This method sets various power levels of the Roller Intake.
         *
         * @param intakePower specifies the intake power.
         * @param ejectPower specifies the eject power.
         * @param retainPower specifies the retain power.
         * @return this parameter object.
         */
        public Params setPowerLevels(double intakePower, double ejectPower, double retainPower)
        {
            this.intakePower = intakePower;
            this.ejectPower = ejectPower;
            this.retainPower = retainPower;
            return this;
        }   //setPowerLevels

        /**
         * This method sets various power levels of the Roller Intake.
         *
         * @param intakeFinishDelay specifies the intake finish delay in seconds.
         * @param ejectFinishDelay specifies the eject finish delay in seconds.
         * @return this parameter object.
         */
        public Params setFinishDelays(double intakeFinishDelay, double ejectFinishDelay)
        {
            this.intakeFinishDelay = intakeFinishDelay;
            this.ejectFinishDelay = ejectFinishDelay;
            return this;
        }   //setFinishDelays

    }   //class Params
 
    /**
     * This class encapsulates all the parameters required to perform the action.
     */
    private static class ActionParams
    {
        boolean intakeAction;
        String owner;
        TrcEvent completionEvent;
        double timeout;
        TrcEvent callbackEvent;

        ActionParams(
            boolean intakeAction, String owner, TrcEvent completionEvent, double timeout, TrcEvent callbackEvent)
        {
            this.intakeAction = intakeAction;
            this.owner = owner;
            this.completionEvent = completionEvent;
            this.timeout = timeout;
            this.callbackEvent = callbackEvent;
        }   //ActionParams

        @Override
        public String toString()
        {
            return "(intakeAction=" + intakeAction +
                   ",owner=" + owner +
                   ",completionEvent=" + completionEvent +
                   ",timeout=" + timeout +
                   ",callbackEvent=" + callbackEvent + ")";
        }   //toString

    }   //class ActionParams

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final Params params;
    private final TrcTimer timer;
    private ActionParams actionParams = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the RollerIntake params.
     */
    public TrcRollerIntake(String instanceName, Params params)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.params = params;

        if (params.frontTriggerParams != null && params.frontTriggerParams.trigger != null)
        {
            params.frontTriggerParams.trigger.enableTrigger(
                TriggerMode.OnBoth, (c)->processTrigger(((AtomicBoolean) c).get(), params.frontTriggerParams));
        }

        if (params.backTriggerParams != null && params.backTriggerParams.trigger != null)
        {
            params.backTriggerParams.trigger.enableTrigger(
                TriggerMode.OnBoth, (c)->processTrigger(((AtomicBoolean) c).get(), params.backTriggerParams));
        }

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
     */
    private void processTrigger(boolean active, TriggerParams triggerParams)
    {
        tracer.traceDebug(
            instanceName, "Trigger: active=%s, triggerParams=%s, actionParams=%s",
            active, triggerParams, actionParams);

        if (actionParams != null)
        {
            if (triggerParams.triggerAction == TriggerAction.StartOnTrigger)
            {
                performAction(actionParams);
            }
            else if (triggerParams.triggerAction == TriggerAction.FinishOnTrigger)
            {
                double finishDelay = actionParams.intakeAction? params.intakeFinishDelay: params.ejectFinishDelay;

                if (finishDelay > 0.0)
                {
                    timer.set(finishDelay, (c)-> finishAction(true));
                }
                else
                {
                    finishAction(true);
                }
            }
        }

        if (triggerParams.triggerCallback != null &&
            (triggerParams.triggerMode == TriggerMode.OnBoth ||
             triggerParams.triggerMode == TriggerMode.OnActive && active ||
             triggerParams.triggerMode == TriggerMode.OnInactive && !active))
        {
            // Caller has registered a callback with a specified trigger mode, call it.
            triggerParams.triggerCallback.notify(triggerParams.callbackContext);
        }
    }   //processTrigger

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
        return params.frontTriggerParams != null &&
               params.frontTriggerParams.trigger != null &&
               params.frontTriggerParams.trigger.getTriggerState();
    }   //getFrontTriggerState

    /**
     * This method returns the back trigger state.
     *
     * @return back trigger state, false if there is no back trigger.
     */
    public boolean getBackTriggerState()
    {
        return params.backTriggerParams != null &&
               params.backTriggerParams.trigger != null &&
               params.backTriggerParams.trigger.getTriggerState();
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
        return params.frontTriggerParams != null && params.frontTriggerParams.trigger != null?
                params.frontTriggerParams.trigger.getSensorValue(): 0.0;
    }   //getFrontSensorValue

    /**
     * This method returns the sensor value read from the back analog sensor.
     *
     * @return back analog sensor value, or zero if there is no back sensor.
     */
    public double getBackSensorValue()
    {
        return params.backTriggerParams != null && params.backTriggerParams.trigger != null?
                params.backTriggerParams.trigger.getSensorValue(): 0.0;
    }   //getBackSensorValue

    /**
     * This method checks if the intake motor is ON.
     *
     * @return true if intake motor is ON, false if open.
     */
    public boolean isOn()
    {
        return params.motor.getPower() != 0.0;
    }   //isOn

    /**
     * This method returns the intake motor current.
     *
     * @return intake motor current.
     */
    public double getCurrent()
    {
        return params.motor.getCurrent();
    }   //getCurrent

    /**
     * This method returns the current intake motor power.
     *
     * @return current intake motor power.
     */
    public double getPower()
    {
        return params.motor.getPower();
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
            params.motor.setPower(owner, delay, power, duration, event);
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
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void intake(String owner, double delay, double duration, TrcEvent event)
    {
        setPower(owner, delay, params.intakePower, duration, event);
    }   //intake

    /**
     * This method starts the motor with intake power and optionally specifies the duration after which to turn off
     * the motor automatically and signal an event if provided.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void intake(String owner, double duration, TrcEvent event)
    {
        setPower(owner, 0.0, params.intakePower, duration, event);
    }   //intake

    /**
     * This method starts the motor with intake power and optionally specifies the duration after which to turn off
     * the motor automatically and signal an event if provided.
     *
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void intake(double duration, TrcEvent event)
    {
        setPower(null, 0.0, params.intakePower, duration, event);
    }   //intake

    /**
     * This method starts the motor with intake power.
     */
    public void intake()
    {
        setPower(null, 0.0, params.intakePower, 0.0, null);
    }   //intake

    /**
     * This method starts the motor with eject power and optionally specifies the duration after which to turn off
     * the motor automatically and signal an event if provided.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void eject(String owner, double delay, double duration, TrcEvent event)
    {
        setPower(owner, delay, params.ejectPower, duration, event);
    }   //eject

    /**
     * This method starts the motor with eject power and optionally specifies the duration after which to turn off
     * the motor automatically and signal an event if provided.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void eject(String owner, double duration, TrcEvent event)
    {
        setPower(owner, 0.0, params.ejectPower, duration, event);
    }   //eject

    /**
     * This method starts the motor with eject power and optionally specifies the duration after which to turn off
     * the motor automatically and signal an event if provided.
     *
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void eject(double duration, TrcEvent event)
    {
        setPower(null, 0.0, params.ejectPower, duration, event);
    }   //eject

    /**
     * This method starts the motor with eject power.
     */
    public void eject()
    {
        setPower(null, 0.0, params.ejectPower, 0.0, null);
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
                instanceName, "FinishAction(completed=%s): isOn=%s, hasObject=%s, actionParams=%s",
                completed, isOn(), gotObject, actionParams);
            timer.cancel();
            if (completed)
            {
                double power = actionParams.intakeAction && gotObject? params.retainPower: 0.0;
                params.motor.setPower(actionParams.owner, 0.0, power, 0.0, null);
            }
            else
            {
                // Operation was canceled, cancel operation to release ownership if any.
                params.motor.cancel();
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

            if (actionParams.callbackEvent != null)
            {
                actionParams.callbackEvent.signal();
                actionParams.callbackEvent = null;
            }
            actionParams = null;
        }
        else if (!completed)
        {
            // We are canceling but there was no pending auto action, just stop the motor.
            params.motor.cancel();
        }
    }   //finishAction

    /**
     * This method cancels the auto-assist operation and to clean up. It is called by enableAutoAssist to cancel a
     * previous operation or if the auto-assist has set a timeout and it has expired. Auto-assist will not be canceled
     * even if the sensor trigger caused it to grab an object. If a timeout is not set, auto-assist remains enabled
     * and can auto grab an object over and over again until the user calls this method to cancel the operation.
     *
     * @param context not used.
     */
    private void actionTimedOut(Object context)
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
     */
    private void performAction(Object context)
    {
        ActionParams ap = (ActionParams) context;
        boolean gotObject = hasObject();
        boolean actionPending = false;

        if (ap.intakeAction && !gotObject)
        {
            // We are intaking but we don't have the object yet.
            tracer.traceDebug(instanceName, "Start Intake.");
            params.motor.setPower(ap.owner, 0.0, params.intakePower, 0.0, null);
            actionPending = true;
        }
        else if (!ap.intakeAction && gotObject)
        {
            // We are ejecting and we still have the object.
            tracer.traceDebug(instanceName, "Start Eject.");
            params.motor.setPower(ap.owner, 0.0, params.ejectPower, 0.0, null);
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
    }   //performAction

    /**
     * This method enables auto operation. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param intakeAction specifies true if the action is intake, false if eject.
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param finishDelay specifies the delay in seconds between sensor trigger and finishing the operation, can
     *        be 0.0 for no delay. This is useful to make sure the grabber has a good grasp of the object before
     *        we turn off the motor.
     * @param completionEvent specifies the event to signal when the action is completed.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     * @param completionCallback specifies the method to call when the action is completed or canceled, can be null if
     *        not provided.
     * @param callbackContext specifies the context object to be passed back to the callback, can be null if none.
     */
    private void autoAction(
        boolean intakeAction, String owner, double delay, double finishDelay, TrcEvent completionEvent, double timeout,
        TrcEvent.Callback completionCallback, Object callbackContext)
    {
        if ((params.frontTriggerParams == null || params.frontTriggerParams.triggerAction == TriggerAction.NoAction) &&
            (params.backTriggerParams == null || params.backTriggerParams.triggerAction == TriggerAction.NoAction))
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
            // If there is a triggerCallback, set it up to callback on the same thread of this caller.
            TrcEvent callbackEvent = null;
            if (completionCallback != null)
            {
                callbackEvent = new TrcEvent(instanceName + ".callback");
                callbackEvent.setCallback(completionCallback, callbackContext);
            }
            actionParams = new ActionParams(intakeAction, owner, completionEvent, timeout, callbackEvent);
            if (delay > 0.0)
            {
                timer.set(delay, this::performAction, actionParams);
            }
            else
            {
                performAction(actionParams);
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
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     * @param triggerCallback specifies the method to call when a trigger occurred, can be null if not provided.
     * @param callbackContext specifies the context object to be passed back to the callback, can be null if none.
     */
    public void autoIntake(
        String owner, double delay, TrcEvent event, double timeout, TrcEvent.Callback triggerCallback,
        Object callbackContext)
    {
        autoAction(true, owner, delay, params.intakeFinishDelay, event, timeout, triggerCallback, callbackContext);
    }   //autoIntake

    /**
     * This method enables auto intake. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param finishDelay specifies the delay in seconds between sensor trigger and finishing the operation, can
     *        be 0.0 for no delay. This is useful to make sure the grabber has a good grasp of the object before
     *        we turn off the motor.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     */
    public void autoIntake(String owner, double delay, double finishDelay, TrcEvent event, double timeout)
    {
        autoAction(true, owner, delay, finishDelay, event, timeout, null, null);
    }   //autoIntake

    /**
     * This method enables auto intake. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param finishDelay specifies the delay in seconds between sensor trigger and finishing the operation, can
     *        be 0.0 for no delay. This is useful to make sure the grabber has a good grasp of the object before
     *        we turn off the motor.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     */
    public void autoIntake(String owner, double finishDelay, TrcEvent event, double timeout)
    {
        autoAction(true, owner, 0.0, finishDelay, event, timeout, null, null);
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
        autoAction(true, owner, 0.0, 0.0, null, 0.0, null, null);
    }   //autoIntake

    /**
     * This method enables auto eject. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param finishDelay specifies the delay in seconds between sensor trigger and finishing the operation, can
     *        be 0.0 for no delay. This is useful to make sure the grabber has a good grasp of the object before
     *        we turn off the motor.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     * @param triggerCallback specifies the method to call when a trigger occurred, can be null if not provided.
     * @param callbackContext specifies the context object to be passed back to the callback, can be null if none.
     */
    public void autoEject(
        String owner, double delay, double finishDelay, TrcEvent event, double timeout,
        TrcEvent.Callback triggerCallback, Object callbackContext)
    {
        autoAction(false, owner, delay, finishDelay, event, timeout, triggerCallback, callbackContext);
    }   //autoEject

    /**
     * This method enables auto eject. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param finishDelay specifies the delay in seconds between sensor trigger and finishing the operation, can
     *        be 0.0 for no delay. This is useful to make sure the grabber has a good grasp of the object before
     *        we turn off the motor.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     */
    public void autoEject(String owner, double delay, double finishDelay, TrcEvent event, double timeout)
    {
        autoAction(false, owner, delay, finishDelay, event, timeout, null, null);
    }   //autoEject

    /**
     * This method enables auto eject. It allows the caller to start monitoring the trigger sensor for the object
     * in the grabber. If the operation is completed, it will automatically stop the motor. If an event is provided,
     * it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param finishDelay specifies the delay in seconds between sensor trigger and finishing the operation, can
     *        be 0.0 for no delay. This is useful to make sure the grabber has a good grasp of the object before
     *        we turn off the motor.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     */
    public void autoEject(String owner, double finishDelay, TrcEvent event, double timeout)
    {
        autoAction(false, owner, 0.0, finishDelay, event, timeout, null, null);
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
        autoAction(false, owner, 0.0, 0.0, null, 0.0, null, null);
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
