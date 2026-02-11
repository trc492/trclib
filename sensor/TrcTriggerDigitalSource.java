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

package trclib.sensor;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements a trigger using digital sensor source. A digital source trigger monitors the digital sensor
 * source and notifies the callback handler if it changes state.
 */
public class TrcTriggerDigitalSource implements TrcTrigger
{
    /**
     * This class encapsulates the trigger state. Access to this object must be thread safe (i.e. needs to be
     * synchronized).
     */
    private static class TriggerState
    {
        volatile boolean sensorState;
        volatile boolean triggerEnabled;
        TriggerMode triggerMode;
        TrcEvent triggerEvent;
        TrcEvent.Callback triggerCallback;
        Thread callbackThread;

        TriggerState(boolean state, boolean enabled)
        {
            this.sensorState = state;
            this.triggerEnabled = enabled;
            this.triggerMode = null;
            this.triggerEvent = null;
            this.triggerCallback = null;
            this.callbackThread = null;
        }   //TriggerState

        @Override
        public String toString()
        {
            return "(state=" + sensorState +
                   ",enabled=" + triggerEnabled + ")";
        }   //toString

    }   //class TriggerState

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final BooleanSupplier digitalSource;
    private final TrcTimer timer;
    private final TriggerState triggerState;
    private final AtomicBoolean callbackContext;
    private final TrcTaskMgr.TaskObject triggerTaskObj;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param digitalSource specifies the source that supplies the digital state.
     */
    public TrcTriggerDigitalSource(String instanceName, BooleanSupplier digitalSource)
    {
        if (digitalSource == null)
        {
            throw new IllegalArgumentException("Digital Source cannot be null.");
        }

        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.digitalSource = digitalSource;
        timer = new TrcTimer(instanceName);
        triggerState = new TriggerState(digitalSource.getAsBoolean(), false);
        callbackContext = new AtomicBoolean();
        triggerTaskObj = TrcTaskMgr.createTask(instanceName + ".triggerTask", this::triggerTask);
    }   //TrcTriggerDigitalInput

    /**
     * This method returns the instance name and its state.
     *
     * @return instance name and state.
     */
    @Override
    public String toString()
    {
        String str;

        synchronized (triggerState)
        {
            str = instanceName + "=" + triggerState;
        }

        return str;
    }   //toString

    //
    // Implements TrcTrigger interface.
    //

    /**
     * This method sets the trigger parameters.
     *
     * @param mode specifies the trigger mode.
     * @param event specifies the event to signal when trigger occurs.
     */
    private void setTriggerParams(TriggerMode mode, TrcEvent event)
    {
        synchronized (triggerState)
        {
            triggerState.triggerMode = mode;
            triggerState.triggerEvent = event;
            triggerState.triggerCallback = null;
            triggerState.callbackThread = null;
        }
    }   //setTriggerParams

    /**
     * This method sets the trigger parameters.
     *
     * @param mode specifies the trigger mode.
     * @param callback specifies the callback method to call when trigger occurs.
     */
    private void setTriggerParams(TriggerMode mode, TrcEvent.Callback callback)
    {
        synchronized (triggerState)
        {
            triggerState.triggerMode = mode;
            triggerState.triggerEvent = new TrcEvent(instanceName + ".triggerEvent");
            triggerState.triggerCallback = callback;
            triggerState.callbackThread = Thread.currentThread();
        }
    }   //setTriggerParams

    /**
     * This method arms/disarms the trigger. It enables/disables the task that monitors the sensor value.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    private void setEnabled(boolean enabled)
    {
        synchronized (triggerState)
        {
            // Cancel previous trigger delay timer if there is one.
            timer.cancel();
            if (enabled)
            {
                triggerState.triggerEvent.clear();
                triggerState.sensorState = digitalSource.getAsBoolean();
                triggerTaskObj.registerTask(TrcTaskMgr.TaskType.PRE_PERIODIC_TASK);
            }
            else
            {
                triggerTaskObj.unregisterTask();
                triggerState.triggerEvent.cancel();
                triggerState.triggerEvent = null;
            }
            triggerState.triggerEnabled = enabled;
            tracer.traceDebug(instanceName, "enabled=" + enabled + " (state=" + triggerState + ")");
        }
    }   //setEnabled

    /**
     * This method arms the trigger. It enables the task that monitors the sensor value.
     *
     * @param triggerDelay specifies the delay in seconds before enabling trigger, null if none.
     * @param triggerMode specifies the trigger mode that will signal the event.
     * @param event specifies the event to signal when the trigger state changed.
     */
    @Override
    public void enableTrigger(Double triggerDelay, TriggerMode triggerMode, TrcEvent event)
    {
        synchronized (triggerState)
        {
            // Enable trigger only if it's not already enabled.
            if (!triggerState.triggerEnabled)
            {
                setTriggerParams(triggerMode, event);
                if (triggerDelay != null)
                {
                    timer.set(
                        triggerDelay, (context, canceled) ->
                        {
                            if (!canceled) setEnabled(true);
                        });
                }
                else
                {
                    setEnabled(true);
                }
            }
        }
    }   //enableTrigger

    /**
     * This method arms the trigger. It enables the task that monitors the sensor value.
     *
     * @param triggerDelay specifies the delay in seconds before enabling trigger, null if none.
     * @param triggerMode specifies the trigger mode that will trigger a callback.
     * @param callback specifies the callback handler to notify when the trigger state changed.
     */
    @Override
    public void enableTrigger(Double triggerDelay, TriggerMode triggerMode, TrcEvent.Callback callback)
    {
        synchronized (triggerState)
        {
            // Enable trigger only if it's not already enabled.
            if (!triggerState.triggerEnabled)
            {
                setTriggerParams(triggerMode, callback);
                if (triggerDelay != null)
                {
                    timer.set(
                        triggerDelay, (context, canceled) ->
                        {
                            if (!canceled) setEnabled(true);
                        });
                }
                else
                {
                    setEnabled(true);
                }
            }
        }
    }   //enableTrigger

    /**
     * This method disarms the trigger. It disables the task that monitors the sensor value.
     */
    @Override
    public void disableTrigger()
    {
        synchronized (triggerState)
        {
            if (triggerState.triggerEnabled)
            {
                setEnabled(false);
                triggerState.triggerMode = null;
                triggerState.triggerCallback = null;
                triggerState.callbackThread = null;
            }
        }
    }   //disableTrigger

    /**
     * This method checks if the trigger task is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    @Override
    public boolean isEnabled()
    {
        return triggerState.triggerEnabled;
    }   //isEnabled

    /**
     * This method reads the digital sensor state and will return 1.0 for active and 0.0 for inactive.
     *
     * @return 1.0 for active and 0.0 for inactive.
     */
    @Override
    public double getSensorValue()
    {
        return digitalSource.getAsBoolean()? 1.0: 0.0;
    }   //getSensorValue

    /**
     * This method reads the current digital sensor state.
     *
     * @return current sensor state.
     */
    @Override
    public boolean getSensorState()
    {
        return digitalSource.getAsBoolean();
    }   //getSensorState

    /**
     * This method is called periodically to check the current sensor state. If it has changed from the previous
     * state, the triggerCallback will be notified.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void triggerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        boolean currState = digitalSource.getAsBoolean();
        boolean triggered = false;
        boolean prevState = false;
        TrcEvent.Callback callback = null;
        TrcEvent triggerEvent = null;
        Thread callbackThread = null;

        synchronized (triggerState)
        {
            if (currState != triggerState.sensorState)
            {
                prevState = triggerState.sensorState;
                triggerState.sensorState = currState;
                if (triggerState.triggerMode == TriggerMode.OnBoth ||
                    triggerState.triggerMode == TriggerMode.OnActive && currState ||
                    triggerState.triggerMode == TriggerMode.OnInactive && !currState)
                {
                    triggered = true;
                    callback = triggerState.triggerCallback;
                    triggerEvent = triggerState.triggerEvent;
                    callbackThread = triggerState.callbackThread;
                }
            }
        }
        // Do not hold the lock while doing callback.
        if (triggered)
        {
            tracer.traceDebug(instanceName, "changes state " + prevState + "->" + currState);
            if (callback != null)
            {
                callbackContext.set(currState);
                triggerEvent.setCallback(callbackThread, callback, callbackContext);
            }
            triggerEvent.signal();
        }
    }   //triggerTask

}   //class TrcTriggerDigitalSource
