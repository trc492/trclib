/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

import trclib.motor.TrcServo;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;
import trclib.sensor.TrcTrigger;
import trclib.sensor.TrcTrigger.TriggerMode;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent auto-assist servo claw subsystem. It contains one or two servos
 * and optionally a sensor that detects if the object is within grasp of the claw. It provides the autoAssist
 * methods that allow the caller to pickup or dump objects on a press of a button and the claw subsystem will
 * automatically grab the object once it is within grasp. While it provides the auto-assist functionality to pickup
 * or dump objects, it also supports exclusive subsystem access by implementing TrcExclusiveSubsystem. This enables the
 * claw subsystem to be aware of multiple callers' access to the subsystem. While one caller starts the subsystem
 * for an operation, nobody can access it until the previous caller is done with the operation.
 */
public class TrcServoClaw implements TrcExclusiveSubsystem
{
    /**
     * This class contains all the parameters for the servo claw.
     */
    public static class Params
    {
        private TrcServo servo;
        private TrcTrigger sensorTrigger;
        private double openPos = 0.0;
        private double openTime = 0.5;
        private double closePos = 1.0;
        private double closeTime = 0.5;

        /**
         * This method returns the string form of all the parameters.
         *
         * @return string form of all the parameters.
         */
        @Override
        public String toString()
        {
            return "servo=" + servo +
                   ",sensorTrigger=" + sensorTrigger +
                   ",openPos=" + openPos +
                   ",openTime=" + openTime +
                   ",closePos=" + closePos +
                   ",closeTime=" + closeTime;
        }   //toString

        /**
         * This method sets the servo object for the servo claw.
         *
         * @param servo specifies the servo object.
         * @return this parameter object.
         */
        public Params setServo(TrcServo servo)
        {
            this.servo = servo;
            return this;
        }   //setServo

        /**
         * This method sets the sensor trigger object with optional trigger callback.
         *
         * @param trigger specifies the sensor trigger object.
         * @return this parameter object.
         */
        public Params setSensorTrigger(TrcTrigger trigger)
        {
            this.sensorTrigger = trigger;
            return this;
        }   //setSensorTrigger

        /**
         * This method sets the open/close parameters of the servo claw.
         *
         * @param openPos specifies the open position in physical unit.
         * @param openTime specifies the time in seconds required to open from fully close position.
         * @param closePos specifies the close position in physical unit.
         * @param closeTime specifies the time in seconds required to close from fully open position.
         * @return this parameter object.
         */
        public Params setOpenCloseParams(double openPos, double openTime, double closePos, double closeTime)
        {
            this.openPos = openPos;
            this.openTime = openTime;
            this.closePos = closePos;
            this.closeTime = closeTime;
            return this;
        }   //setOpenCloseParams

    }   //class Params

    /**
     * This class encapsulates all the parameters required to perform the action.
     */
    private static class ActionParams
    {
        String owner;
        TrcEvent completionEvent;
        double timeout;
        TrcEvent callbackEvent;

        ActionParams(String owner, TrcEvent completionEvent, double timeout, TrcEvent callbackEvent)
        {
            this.owner = owner;
            this.completionEvent = completionEvent;
            this.timeout = timeout;
            this.callbackEvent = callbackEvent;
        }   //ActionParams

        @Override
        public String toString()
        {
            return "(owner=" + owner +
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
     * @param params specifies the servo claw parameters.
     */
    public TrcServoClaw(String instanceName, Params params)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.params = params;
        timer = new TrcTimer(instanceName);
    }   //TrcServoClaw

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
     * This method returns the current claw position.
     *
     * @return current claw servo position.
     */
    public double getPosition()
    {
        return params.servo.getPosition();
    }   //getPosition

    /**
     * This method checks if the claw is closed.
     *
     * @return true if claw is closed, false if open.
     */
    public boolean isClosed()
    {
        return params.servo.getPosition() == params.closePos;
    }   //isClosed

    /**
     * This method sets the claw position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *                 setPhysicalRange is called with the degree range.
     * @param event specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(
        String owner, double delay, double position, TrcEvent event, double timeout)
    {
        tracer.traceDebug(
            instanceName, "owner=%s,delay=%.3f,pos=%.3f,event=%s,timeout=%.3f", owner, delay, position, event, timeout);
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
            params.servo.setPosition(owner, delay, position, event, timeout);
        }
    }   //setPosition

    /**
     * This method sets the servo claw to its open position and signals the given event after the open time
     * has expired. It cancels the previous pending auto operation if any.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay.
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void open(String owner, double delay, TrcEvent event)
    {
        setPosition(owner, delay, params.openPos, event, params.openTime);
    }   //open

    /**
     * This method sets the servo claw to its open position and signals the given event after the open time
     * has expired. It cancels the previous pending auto operation if any.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void open(String owner, TrcEvent event)
    {
        setPosition(owner, 0.0, params.openPos, event, params.openTime);
    }   //open

    /**
     * This method sets the servo claw to its open position and signals the given event after the open time
     * has expired. It cancels the previous pending auto operation if any.
     *
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void open(TrcEvent event)
    {
        setPosition(null, 0.0, params.openPos, event, params.openTime);
    }   //open

    /**
     * This method sets the servo claw to its open position. It cancels the previous pending auto operation if any.
     */
    public void open()
    {
        setPosition(null, 0.0, params.openPos, null, 0.0);
    }   //open

    /**
     * This method sets the servo claw to its close position and signals the given event after the close time
     * has expired. It cancels the previous pending auto operation if any.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay.
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void close(String owner, double delay, TrcEvent event)
    {
        setPosition(owner, delay, params.closePos, event, params.closeTime);
    }   //close

    /**
     * This method sets the servo claw to its close position and signals the given event after the close time
     * has expired. It cancels the previous pending auto operation if any.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void close(String owner, TrcEvent event)
    {
        setPosition(owner, 0.0, params.closePos, event, params.closeTime);
    }   //close

    /**
     * This method sets the servo claw to its close position and signals the given event after the close time
     * has expired. It cancels the previous pending auto operation if any.
     *
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void close(TrcEvent event)
    {
        setPosition(null, 0.0, params.closePos, event, params.closeTime);
    }   //close

    /**
     * This method sets the servo claw to its close position. It cancels the previous pending auto operation if any.
     */
    public void close()
    {
        setPosition(null, 0.0, params.closePos, null, 0.0);
    }   //close

    /**
     * This method is called to finish and clean up the action.
     *
     * @param completed specifies true to complete the action, false to cancel.
     */
    private void finishAction(boolean completed)
    {
        // Do clean up only if auto-assist is enabled.
        if (actionParams != null)
        {
            boolean clawClosed = isClosed();
            tracer.traceDebug(
                instanceName, "FinishAction(completed=%s): closed=%s,hasObject=%s,actionParams=%s",
                completed, clawClosed, hasObject(), actionParams);

            timer.cancel();
            params.sensorTrigger.disableTrigger();
            if (!completed)
            {
                // Operation was canceled, cancel operation to release ownership if any.
                params.servo.cancel(actionParams.owner);
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
    }   //finishAction

    /**
     * This method cancels the auto-assist operation and to clean up. It is called by autoAssistGrab to cancel a
     * previous operation or if the auto-assist has set a timeout and it has expired. Auto-assist will not be canceled
     * even if the sensor trigger caused it to grab an object. If a timeout is not set, auto-assist remains enabled
     * and can auto grab an object over and over again until the user cancels the operation.
     *
     * @param context not used.
     * @param canceled not used.
     */
    private void actionTimedOut(Object context, boolean canceled)
    {
        tracer.traceDebug(instanceName, "Auto action timed out.");
        finishAction(hasObject());
    }   //actionTimedOut

    /**
     * This method is called when the claw sensor is triggered.
     *
     * @param context not used.
     * @param canceled specifies true if trigger is disabled.
     */
    private void sensorTriggerCallback(Object context, boolean canceled)
    {
        if (!canceled)
        {
            if (getTriggerState())
            {
                tracer.traceDebug(instanceName, "Triggered: callbackEvent=" + actionParams.callbackEvent);
                // If callback is provided, callback is responsible for grabbing the object.
                if (actionParams.callbackEvent == null)
                {
                    // There is no callback, grab the object ourselves and finish the operation.
                    params.servo.setPosition(actionParams.owner, 0.0, params.closePos, null, 0.0);
                    finishAction(true);
                }
                else
                {
                    // There is a callback, signal the callback. It is the responsibility of the caller to finish the
                    // operation by calling cancel.
                    actionParams.callbackEvent.signal();
                }
            }
        }
    }   //sensorTriggerCallback

    /**
     * This method performs the auto action which is to close the claw if it was open and the object is in
     * proximity or to open the claw if it was close and it doesn't have the object. It arms the sensor trigger
     * to detect the object for auto grabbing. If there is a timeout, it arms the timeout timer for canceling the
     * auto action when the timer expires.
     *
     * @param context specifies the action parameters.
     * @param canceled specifies true if canceled.
     */
    private void performAction(Object context, boolean canceled)
    {
        if (!canceled)
        {
            ActionParams ap = (ActionParams) context;
            boolean triggered = getTriggerState();
            boolean clawClosed = isClosed();
            boolean finished = false;

            tracer.traceDebug(
                instanceName, "AutoAssistGrab: clawClosed=%s, sensorTriggered=%s, params=%s",
                clawClosed, triggered, params);
            if (triggered)
            {
                if (!clawClosed && ap.callbackEvent == null)
                {
                    // Claw is open, the object is detected and there is no callback, grab the object.
                    tracer.traceDebug(instanceName, "Object already detected, grab it!");
                    params.servo.setPosition(ap.owner, 0.0, params.closePos, null, 0.0);
                }
                finished = true;
            }
            else if (clawClosed)
            {
                // Claw is close but has no object, open it to prepare for grabbing.
                tracer.traceDebug(instanceName, "Claws are closed, open it back up.");
                params.servo.setPosition(ap.owner, 0.0, params.openPos, null, 0.0);
            }

            if (finished)
            {
                // Either already grabbed an object or object is detected and we have a callback, finish the action.
                tracer.traceDebug(instanceName, "Already has object, finish the operation.");
                finishAction(true);
            }
            else
            {
                // Arm the sensor trigger as long as AutoAssist is enabled.
                tracer.traceDebug(instanceName, "Arm sensor trigger.");
                params.sensorTrigger.enableTrigger(TriggerMode.OnBoth, this::sensorTriggerCallback);
                if (ap.timeout > 0.0)
                {
                    // Set a timeout and cancel auto-assist if timeout has expired.
                    tracer.traceDebug(instanceName, "Set timeout " + ap.timeout + " sec");
                    timer.set(ap.timeout, this::actionTimedOut, null);
                }
            }
        }
    }   //performAction

    /**
     * This method arms the trigger callback. This is useful in the trigger callback handler that if it decided
     * the trigger is a false trigger, it can ignore the trigger and re-arm the trigger callback. Note that this
     * is intended to be called from the trigger callback handler to re-arm the trigger callback. If it is called
     * somewhere else and there was no pending auto operation or no callback, this call will fail and will return
     * false.
     *
     * @param triggerCallback specifies the method to call when a trigger occurred, can be null if not provided.
     *        If triggerCallback is provided, it is the responsibility of triggerCallback to close the claw to
     *        grab the object and to cancel the operation. In other words, autoGrab mode will not end automatically.
     * @param callbackContext specifies the context object to be passed back to the callback, can be null if none.
     * @return true if rearm is successful, false otherwise.
     */
    public boolean armTriggerCallback(TrcEvent.Callback triggerCallback, Object callbackContext)
    {
        boolean success = false;

        tracer.traceDebug(instanceName, "actionParams=%s", actionParams);
        if (actionParams != null && actionParams.callbackEvent != null)
        {
            actionParams.callbackEvent.setCallback(triggerCallback, callbackContext);
        }

        return success;
    }   //armTriggerCallback

    /**
     * This method enables auto grabbing. It allows the caller to start monitoring the trigger sensor for the object
     * in the vicinity. If the object is within grasp, it will automatically grab the object. If an event is
     * provided, it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the claw subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     * @param triggerCallback specifies the method to call when a trigger occurred, can be null if not provided.
     *        If triggerCallback is provided, it is the responsibility of triggerCallback to close the claw to
     *        grab the object and to cancel the operation. In other words, autoGrab mode will not end automatically.
     * @param callbackContext specifies the context object to be passed back to the callback, can be null if none.
     */
    public void autoGrab(
        String owner, double delay, TrcEvent event, double timeout, TrcEvent.Callback triggerCallback,
        Object callbackContext)
    {
        if (params.sensorTrigger == null)
        {
            throw new RuntimeException("Must have sensor to perform Auto Operation.");
        }
        // This is an auto operation, make sure the caller has ownership.
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, event, tracer);
        if (releaseOwnershipEvent != null) event = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            // In case there is an existing auto operation still pending, cancel it first.
            finishAction(false);
            // If there is a triggerCallback, set it up to callback on the same thread of this caller.
            TrcEvent callbackEvent = null;
            if (triggerCallback != null)
            {
                callbackEvent = new TrcEvent(instanceName + ".callback");
                callbackEvent.setCallback(triggerCallback, callbackContext);
            }
            actionParams = new ActionParams(owner, event, timeout, callbackEvent);
            if (delay > 0.0)
            {
                timer.set(delay, this::performAction, actionParams);
            }
            else
            {
                performAction(actionParams, false);
            }
        }
    }   //autoGrab

    /**
     * This method enables auto grabbing. It allows the caller to start monitoring the trigger sensor for the object
     * in the vicinity. If the object is within grasp, it will automatically grab the object. If an event is
     * provided, it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the claw subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *        must call hasObject() to figure out if it has given up.
     */
    public void autoGrab(String owner, double delay, TrcEvent event, double timeout)
    {
        autoGrab(owner, delay, event, timeout, null,  null);
    }   //autoGrab

    /**
     * This method cancels the auto-assist operation and to clean up. It is called by the user for canceling the
     * operation.
     */
    public void cancel()
    {
        finishAction(false);
    }   //cancel

    /**
     * This method returns the sensor value read from the analog sensor.
     *
     * @return analog sensor value.
     */
    public double getSensorValue()
    {
        return params.sensorTrigger != null? params.sensorTrigger.getSensorValue(): 0.0;
    }   //getSensorValue

    /**
     * This method returns the trigger state read from the sensor.
     *
     * @return sensor trigger state.
     */
    public boolean getTriggerState()
    {
        return params.sensorTrigger != null && params.sensorTrigger.getTriggerState();
    }   //getTriggerState

    /**
     * This method checks if the claw has the object.
     *
     * @return true if claw has the object, false otherwise.
     */
    public boolean hasObject()
    {
        return isClosed() && getTriggerState();
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

}   //class TrcServoClaw
