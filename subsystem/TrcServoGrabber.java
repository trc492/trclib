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
import trclib.dataprocessor.TrcTrigger;
import trclib.dataprocessor.TrcTrigger.TriggerMode;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent auto-assist servo grabber subsystem. It contains one or two servos
 * and optionally a sensor that detects if the object is within grasp of the grabber. It provides the autoAssist
 * methods that allow the caller to pickup or dump objects on a press of a button and the grabber subsystem will
 * automatically grab the object once it is within grasp. While it provides the auto-assist functionality to pickup
 * or dump objects, it also supports exclusive subsystem access by implementing TrcExclusiveSubsystem. This enables the
 * grabber subsystem to be aware of multiple callers' access to the subsystem. While one caller starts the subsystem
 * for an operation, nobody can access it until the previous caller is done with the operation.
 */
public class TrcServoGrabber implements TrcExclusiveSubsystem
{
    /**
     * This class contains all the parameters related to the servo grabber.
     */
    public static class Parameters
    {
        public double maxStepRate = 0.0;
        public boolean servo1Inverted = false;
        public boolean servo2Inverted = false;
        public boolean triggerInverted = false;
        public Double triggerThreshold = null;
        public Double hasObjectThreshold = null;
        public double openPos = 0.0;
        public double openTime = 0.5;
        public double closePos = 1.0;
        public double closeTime = 0.5;

        /**
         * This method returns the string form of all the parameters.
         *
         * @return string form of all the parameters.
         */
        @Override
        public String toString()
        {
            return "maxStepRate=" + maxStepRate +
                   ",servosInverted(" + servo1Inverted + "," + servo2Inverted + ")" +
                   ",triggerInverted=" + triggerInverted +
                   ",triggerThreshold=" + triggerThreshold +
                   ",hasObjThreshold=" + hasObjectThreshold +
                   ",openPos=" + openPos +
                   ",openTime=" + openTime +
                   ",closePos=" + closePos +
                   ",closeTime=" + closeTime;
        }   //toString

        /**
         * This method sets the maximum stepping rate of the servo. This enables setPower to speed control the servo.
         *
         * @param maxStepRate specifies the maximum stepping rate (physicalPos/sec).
         * @return this parameter object.
         */
        public Parameters setMaxStepRate(double maxStepRate)
        {
            this.maxStepRate = maxStepRate;
            return this;
        }   //setMaxStepRate

        /**
         * This mmethod sets servos to be inverted. It changes the direction of the servo movement.
         *
         * @param servo1Inverted specifies true to invert servo1 direction, false otherwise.
         * @param servo2Inverted specifies true to invert servo2 direction, false otherwise.
         * @return this parameter object.
         */
        public Parameters setServoInverted(boolean servo1Inverted, boolean servo2Inverted)
        {
            this.servo1Inverted = servo1Inverted;
            this.servo2Inverted = servo2Inverted;
            return this;
        }   //setServoInverted

        /**
         * This method sets the trigger to be inverted. If it is an analog trigger, inverted means triggering when
         * sensor value is lower than threshold. If it is a digital trigger, inverted means triggering on inactive
         * state.
         *
         * @param inverted specifies true to invert the trigger, false otherwise.
         * @return this parameter object.
         */
        public Parameters setTriggerInverted(boolean inverted)
        {
            this.triggerInverted = inverted;
            return this;
        }   //setTriggerInverted

        /**
         * This method sets the analog sensor threshold value.
         *
         * @param triggerThreshold specifies the trigger threshold value.
         * @param hasObjectThreshold specifies the threshold value to detect object possession.
         * @return this parameter object.
         */
        public Parameters setThresholds(double triggerThreshold, double hasObjectThreshold)
        {
            this.triggerThreshold = triggerThreshold;
            this.hasObjectThreshold = hasObjectThreshold;
            return this;
        }   //setThresholds

        /**
         * This method sets the open parameters of the servo grabber.
         *
         * @param openPos specifies the open position in physical unit
         * @param openTime specifies the time in seconds required to open from a full close position.
         * @return this parameter object.
         */
        public Parameters setOpenParams(double openPos, double openTime)
        {
            this.openPos = openPos;
            this.openTime = openTime;
            return this;
        }   //setOpenParams

        /**
         * This method sets the close parameters of the servo grabber.
         *
         * @param closePos specifies the close position in physical unit
         * @param closeTime specifies the time in seconds required to close from a full open position.
         * @return this parameter object.
         */
        public Parameters setCloseParams(double closePos, double closeTime)
        {
            this.closePos = closePos;
            this.closeTime = closeTime;
            return this;
        }   //setCloseParams

    }   //class Parameters

    /**
     * This class encapsulates all the parameters required to perform the action.
     */
    private static class ActionParams
    {
        String owner;
        TrcEvent event;
        double timeout;

        ActionParams(String owner, TrcEvent event, double timeout)
        {
            this.owner = owner;
            this.event = event;
            this.timeout = timeout;
        }   //ActionParams

        @Override
        public String toString()
        {
            return "(owner=" + owner + ",event=" + event + ",timeout=" + timeout + ")";
        }   //toString

    }   //class ActionParams

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcServo servo;
    private final Parameters params;
    private final TrcTrigger sensorTrigger;
    private final TrcEvent.Callback triggerCallback;
    private final TrcTimer timer;
    private ActionParams actionParams = null;
    private boolean grabberClosed = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param servo1 specifies the first servo object.
     * @param servo2 specifies the second servo object, can be null if none.
     * @param params specifies the parameters to set up the servo grabber.
     * @param sensorTrigger specifies the sensor trigger object, can be null if none.
     * @param triggerCallback specifies the callback handler when the sensor is triggered, null if no sensorTrigger.
     */
    public TrcServoGrabber(
        String instanceName, TrcServo servo1, TrcServo servo2, Parameters params, TrcTrigger sensorTrigger,
        TrcEvent.Callback triggerCallback)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.servo = servo1;
        this.params = params;
        this.sensorTrigger = sensorTrigger;
        this.triggerCallback = triggerCallback;

        servo1.setInverted(params.servo1Inverted);
        if (servo2 != null)
        {
            servo2.setInverted(params.servo2Inverted);
            servo2.follow(servo1);
        }

        if (params.maxStepRate != 0.0)
        {
            servo.setMaxStepRate(params.maxStepRate);
        }

        timer = new TrcTimer(instanceName);
    }   //TrcServoGrabber

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param servo specifies the first servo object.
     * @param params specifies the parameters to set up the servo grabber.
     * @param sensorTrigger specifies the sensor trigger object, can be null if none.
     * @param triggerCallback specifies the callback handler when the sensor is triggered, null if no sensorTrigger.
     */
    public TrcServoGrabber(
        String instanceName, TrcServo servo, Parameters params, TrcTrigger sensorTrigger,
        TrcEvent.Callback triggerCallback)
    {
        this(instanceName, servo, null, params, sensorTrigger, triggerCallback);
    }   //TrcServoGrabber

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param servo specifies the first servo object.
     * @param params specifies the parameters to set up the servo grabber.
     */
    public TrcServoGrabber(String instanceName, TrcServo servo, Parameters params)
    {
        this(instanceName, servo, null, params, null, null);
    }   //TrcServoGrabber

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
     * This method returns the current grabber position.
     *
     * @return current grabber servo position.
     */
    public double getPosition()
    {
        return servo.getPosition();
    }   //getPosition

    /**
     * This method sets the grabber position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *                 setPhysicalRange is called with the degree range.
     * @param event specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     * @param cancelAutoAssist specifies true to cancel previous auto-assist operation if any.
     */
    private void setPosition(
        String owner, double delay, double position, TrcEvent event, double timeout, boolean cancelAutoAssist)
    {
        if (event != null)
        {
            event.clear();
        }

        tracer.traceDebug(
            instanceName,
            "owner=" + owner +
            ", delay=" + delay +
            ", pos=" + position +
            ", event=" + event +
            ", timeout=" + timeout +
            ", cancelAutoAssist=" + cancelAutoAssist);

        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, event, tracer);
        if (releaseOwnershipEvent != null) event = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            if (cancelAutoAssist)
            {
                cancelAction(null);
            }

            servo.setPosition(owner, delay, position, event, timeout);
        }
    }   //setPosition

    /**
     * This method sets the grabber position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *                 setPhysicalRange is called with the degree range.
     * @param event specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(String owner, double delay, double position, TrcEvent event, double timeout)
    {
        setPosition(owner, delay, position, event, timeout, true);
    }   //setPosition

    /**
     * This method sets the grabber position.
     *
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *                 setPhysicalRange is called with the degree range.
     * @param event specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(double delay, double position, TrcEvent event, double timeout)
    {
        setPosition(null, delay, position, event, timeout, true);
    }   //setPosition

    /**
     * This method sets the grabber position.
     *
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *                 setPhysicalRange is called with the degree range.
     * @param event specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(double position, TrcEvent event, double timeout)
    {
        setPosition(null, 0.0, position, event, timeout, true);
    }   //setPosition

    /**
     * This method sets the grabber position.
     *
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *                 setPhysicalRange is called with the degree range.
     */
    public void setPosition(double position)
    {
        setPosition(null, 0.0, position, null, 0.0, true);
    }   //setPosition

    /**
     * This method sets the servo grabber to its open position and signals the given event after the open time
     * has expired. If cancelAutoAssist is true, cancel the pending autoAssist operation if any.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param event specifies the event to be signaled after specified time has expired.
     * @param cancelAutoAssist specifies true to cancel previous auto-assist operation if any.
     */
    public void open(String owner, TrcEvent event, boolean cancelAutoAssist)
    {
        setPosition(owner, 0.0, params.openPos, event, params.openTime, cancelAutoAssist);
        grabberClosed = false;
    }   //open

    /**
     * This method sets the servo grabber to its open position and signals the given event after the open time
     * has expired.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void open(String owner, TrcEvent event)
    {
        open(owner, event, true);
    }   //open

    /**
     * This method sets the servo grabber to its open position and signals the given event after the open time
     * has expired.
     *
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void open(TrcEvent event)
    {
        open(null, event, true);
    }   //open

    /**
     * This method sets the servo grabber to its open position.
     */
    public void open()
    {
        open(null, null, true);
    }   //open

    /**
     * This method sets the servo grabber to its close position and signals the given event after the close time
     * has expired. If cancelAutoAssist is true, cancel the pending autoAssist operation if any.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param event specifies the event to be signaled after specified time has expired.
     * @param cancelAutoAssist specifies true to cancel previous auto-assist operation if any.
     */
    public void close(String owner, TrcEvent event, boolean cancelAutoAssist)
    {
        setPosition(owner, 0.0, params.closePos, event, params.closeTime, cancelAutoAssist);
        grabberClosed = true;
    }   //close

    /**
     * This method sets the servo grabber to its close position and signals the given event after the close time
     * has expired.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void close(String owner, TrcEvent event)
    {
        close(owner, event, true);
    }   //close
    /**
     * This method sets the servo grabber to its close position and signals the given event after the close time
     * has expired.
     *
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void close(TrcEvent event)
    {
        close(null, event, true);
    }   //close

    /**
     * This method sets the servo grabber to its close position.
     */
    public void close()
    {
        close(null, null, true);
    }   //close

    /**
     * This method enables auto-assist grabbing which is to close the grabber if it was open and the object is in
     * proximity or to open the grabber if it was close and it doesn't have the object. It arms the sensor trigger
     * to detect the object for auto grabbing. If there is a timeout, it arms the timeout timer for canceling the
     * auto-assist grabbing operation when the timer expires.
     *
     * @param context specifies the action parameters.
     */
    private void enableAction(Object context)
    {
        ActionParams ap = (ActionParams) context;
        boolean inProximity = objectInProximity();
        boolean grabbedObject = false;

        tracer.traceDebug(
            instanceName,
            "EnableAutoAssist: grabberClosed=" + grabberClosed +
            ", inProximity=" + inProximity +
            ", params=" + ap);
        if (grabberClosed && !inProximity)
        {
            // Grabber is close but has no object, open it to prepare for grabbing.
            open(ap.owner, null, false);
        }
        else if (!grabberClosed && inProximity)
        {
            // Grabber is open but the object is near by, grab it and signal the event.
            close(ap.owner, ap.event, false);
            grabbedObject = true;
        }
        // Arm the sensor trigger as long as AutoAssist is enabled.
        sensorTrigger.enableTrigger(TriggerMode.OnBoth, triggerCallback);
        if (ap.timeout > 0.0)
        {
            if (grabbedObject)
            {
                // User has set a timeout and we grabbed an object already, cancel auto-assist.
                cancelAction(null);
            }
            else
            {
                // Set a timeout and cancel auto-assist if timeout has expired.
                timer.set(ap.timeout, this::cancelAction, null);
            }
        }
    }   //enableAction

    /**
     * This method cancels the auto-assist operation and to clean up. It is called by enableAutoAssist to cancel a
     * previous operation or if the auto-assist has set a timeout and it has expired. Auto-assist will not be canceled
     * even if the sensor trigger caused it to grab an object. If a timeout is not set, auto-assist remains enabled
     * and can auto grab an object over and over again until the user calls this method to cancel the operation.
     *
     * @param context not used.
     */
    private void cancelAction(Object context)
    {
        // Do clean up only if auto-assist is enabled.
        if (actionParams != null)
        {
            boolean hasObject = hasObject();

            tracer.traceDebug(
                instanceName,
                "CancelAutoAssist: grabberClosed=" + grabberClosed +
                ", hasObject=" + hasObject +
                ", params=" + actionParams);
            if (!hasObject && actionParams.event != null)
            {
                actionParams.event.cancel();
            }

            if (grabberClosed)
            {
                open(actionParams.owner, null, false);
            }

            timer.cancel();
            sensorTrigger.disableTrigger();
            actionParams = null;
        }
    }   //cancelAction

    /**
     * This method enables auto-assist grabbing. It allows the caller to start monitoring the trigger sensor for
     * the object in the vicinity. If the object is within grasp, it will automatically grab the object. If an
     * event is provided, it will also signal the event when the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the grabber subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void enableAutoAssist(String owner, double delay, TrcEvent event, double timeout)
    {
        if (sensorTrigger == null)
        {
            throw new RuntimeException("Must have sensor to perform AutoAssist.");
        }
        //
        // This is an auto-assist operation, make sure the caller has ownership.
        //
        if (validateOwnership(owner))
        {
            // In case there is an existing auto-assist still pending, cancel it first.
            cancelAction(null);
            actionParams = new ActionParams(owner, event, timeout);
            if (delay > 0.0)
            {
                timer.set(delay, this::enableAction, actionParams);
            }
            else
            {
                enableAction(actionParams);
            }
        }
    }   //enableAutoAssist

    /**
     * This method cancels the auto-assist operation and to clean up. It is called by the user for canceling the
     * operation.
     */
    public void cancelAutoAssist()
    {
        cancelAction(null);
    }   //cancelAutoAssist

    /**
     * This method returns the sensor value read from the analog sensor.
     *
     * @return analog sensor value.
     */
    public double getSensorValue()
    {
        return sensorTrigger != null? sensorTrigger.getSensorValue(): 0.0;
    }   //getSensorValue

    /**
     * This method returns the sensor state read from the digital sensor.
     *
     * @return digital sensor state.
     */
    public boolean getSensorState()
    {
        return sensorTrigger != null && sensorTrigger.getSensorState();
    }   //getSensorState

    /**
     *
     * This method checks if object is detected in the proximity.
     *
     * @return true if object is detected in the proximity, false otherwise.
     */
    public boolean objectInProximity()
    {
        boolean inProximity = false;

        if (sensorTrigger != null)
        {
            if (params.hasObjectThreshold != null)
            {
                inProximity = getSensorValue() > params.hasObjectThreshold;
            }
            else
            {
                inProximity = getSensorState();
            }

            if (params.triggerInverted)
            {
                inProximity = !inProximity;
            }
        }

        return inProximity;
    }   //objectInProximity

    /**
     * This method checks if the grabber has the object.
     *
     * @return true if grabber has the object, false otherwise.
     */
    public boolean hasObject()
    {
        return grabberClosed && objectInProximity();
    }   //hasObject

    /**
     * This method checks if auto-assist is active.
     *
     * @return true if auto-assist is in progress, false otherwise.
     */
    public boolean isAutoAssistActive()
    {
        return actionParams != null;
    }   //isAutoAssistActive

}   //class TrcServoGrabber
