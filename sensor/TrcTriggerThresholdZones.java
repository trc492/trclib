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

package trclib.sensor;

import java.util.Locale;
import java.util.function.DoubleSupplier;

import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements a Threshold Zones Trigger. It monitors the value source against an array of threshold
 * values. If the sensor reading crosses any of the thresholds in the array, it signals an event or notifies the
 * callback handler so that an action could be performed.
 */
public class TrcTriggerThresholdZones implements TrcTrigger
{
    /**
     * This class encapsulates the trigger state. Access to this object must be thread safe (i.e. needs to be
     * synchronized).
     */
    private static class TriggerState
    {
        volatile double sensorValue;
        volatile int sensorZone;
        volatile int prevZone;
        volatile boolean triggerEnabled;
        TriggerMode triggerMode;
        TrcEvent triggerEvent;
        TrcEvent.Callback triggerCallback;
        Thread callbackThread;

        TriggerState(double sensorValue, int sensorZone, boolean triggerEnabled)
        {
            this.sensorValue = sensorValue;
            this.sensorZone = sensorZone;
            this.prevZone = sensorZone;
            this.triggerEnabled = triggerEnabled;
            this.triggerMode = null;
            this.triggerEvent = null;
            this.triggerCallback = null;
            this.callbackThread = null;
        }   //TriggerState

        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "(value=%f,Zone=%d,prevZone=%d,Enabled=%s)",
                sensorValue, sensorZone, prevZone, triggerEnabled);
        }   //toString

    }   //class TriggerState

    /**
     * This class encapsulates all the info for the the trigger event callback.
     */
    public static class CallbackContext
    {
        public double sensorValue;
        public int prevZone;
        public int currZone;

        @Override
        public String toString()
        {
            return String.format(Locale.US, "(value-%f,prevZone=%d,currZone=%d)", sensorValue, prevZone, currZone);
        }   //toString

    }   //class CallbackContext

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final DoubleSupplier valueSource;
    private final TrcTimer timer;
    private final TriggerState triggerState;
    private final CallbackContext callbackContext;
    private final TrcTaskMgr.TaskObject triggerTaskObj;
    private final double[] thresholdPoints;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param valueSource specifies the interface that implements the value source.
     * @param thresholdPoints specifies an array of threshold points for the trigger.
     */
    public TrcTriggerThresholdZones(
        String instanceName, DoubleSupplier valueSource, double... thresholdPoints)
    {
        if (valueSource == null)
        {
            throw new IllegalArgumentException("ValueSource cannot be null.");
        }

        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.valueSource = valueSource;
        this.thresholdPoints = thresholdPoints;
        timer = new TrcTimer(instanceName);

        double value = getSensorValue();
        triggerState = new TriggerState(value, getValueZone(value), false);
        callbackContext = new CallbackContext();
        triggerTaskObj = TrcTaskMgr.createTask(instanceName + ".triggerTask", this::triggerTask);
    }   //TrcTriggerThresholdZones

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
                triggerState.sensorValue = getSensorValue();
                triggerState.sensorZone = getValueZone(triggerState.sensorValue);
                triggerState.prevZone = triggerState.sensorZone;
                triggerTaskObj.registerTask(TrcTaskMgr.TaskType.PRE_PERIODIC_TASK);
            }
            else
            {
                triggerTaskObj.unregisterTask();
                triggerState.triggerEvent. cancel();
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
     * @param triggerMode specifies the trigger direction that will signal the event. TriggerMode.OnActive will
     *        trigger only when crossing a lower zone to a higher zone. TriggerMode.OnInactive will trigger only when
     *        crossing from a higher zone to a lower zone. TriggerMode.OnBoth will trigger on both directions.
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
     * @param triggerMode specifies the trigger direction that will trigger a callback. TriggerMode.OnActive will
     *        trigger only when crossing a lower zone to a higher zone. TriggerMode.OnInactive will trigger only when
     *        crossing from a higher zone to a lower zone. TriggerMode.OnBoth will trigger on both directions.
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
     * This method reads the current analog sensor value.
     *
     * @return current sensor value.
     */
    @Override
    public double getSensorValue()
    {
        return valueSource.getAsDouble();
    }   //getSensorValue

    /**
     * This method reads the current trigger state. TrcTriggerThresholdZones does not support getTriggerState and
     * will throw an exception.
     *
     * @return current trigger state.
     */
    @Override
    public boolean getTriggerState()
    {
        throw new UnsupportedOperationException("TrcTriggerThresholdZones does not support trigger state.");
    }   //getTriggerState

    /**
     * This method returns the current zone it is in.
     *
     * @return current zone index.
     */
    public int getCurrentZone()
    {
        return triggerState.sensorZone;
    }   //getCurrentZone

    /**
     * This method returns the previous zone it was in.
     *
     * @return previous zone index.
     */
    public int getPreviousZone()
    {
        return triggerState.prevZone;
    }   //getPreviousZone

    /**
     * This method determines the sensor zone with the given sensor value.
     *
     * @param value specifies the sensor value.
     * @return sensor zone the value is in.
     */
    private int getValueZone(double value)
    {
        int zone = -1;

        if (value < thresholdPoints[0])
        {
            zone = 0;
        }
        else
        {
            for (int i = 0; i < thresholdPoints.length - 1; i++)
            {
                if (value >= thresholdPoints[i] && value < thresholdPoints[i + 1])
                {
                    zone = i + 1;
                    break;
                }
            }

            if (zone == -1)
            {
                zone = thresholdPoints.length;
            }
        }
        tracer.traceDebug(instanceName, "value=%f, zone=%d", value, zone);

        return zone;
    }   //getValueZone

    /**
     * This method is called periodically to check the current sensor value against the threshold array to see it
     * crosses any thresholds and the triggerHandler will be notified.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void triggerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        double currValue = getSensorValue();
        int currZone = getValueZone(currValue);
        boolean triggered = false;
        int prevZone = -1;
        TrcEvent.Callback callback = null;
        TrcEvent triggerEvent = null;
        Thread callbackThread = null;

        synchronized (triggerState)
        {
            if (currZone != triggerState.sensorZone)
            {
                //
                // We have crossed to another zone, let's notify somebody.
                //
                prevZone = triggerState.sensorZone;
                triggerState.sensorZone = currZone;
                triggerState.prevZone = prevZone;
                if (triggerState.triggerMode == TriggerMode.OnBoth ||
                    triggerState.triggerMode == TriggerMode.OnActive && prevZone < currZone ||
                    triggerState.triggerMode == TriggerMode.OnInactive && prevZone > currZone)
                {
                    triggered = true;
                    callback = triggerState.triggerCallback;
                    triggerEvent = triggerState.triggerEvent;
                    callbackThread = triggerState.callbackThread;
                }
            }
            triggerState.sensorValue = currValue;
        }
        // Do not hold the lock while doing callback.
        if (triggered)
        {
            tracer.traceDebug(instanceName, "Crossing zones %d->%d (value=%f)", prevZone, currZone, currValue);
            if (callback != null)
            {
                synchronized (callbackContext)
                {
                    callbackContext.sensorValue = currValue;
                    callbackContext.prevZone = prevZone;
                    callbackContext.currZone = currZone;
                }
                triggerEvent.setCallback(callbackThread, callback, callbackContext);
            }
            triggerEvent.signal();
        }
    }   //triggerTask

}   //class TrcTriggerThresholdZones
