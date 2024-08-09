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

import java.util.Arrays;
import java.util.Locale;

import trclib.dataprocessor.TrcValueSource;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

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

        TriggerState(double sensorValue, int sensorZone, boolean triggerEnabled)
        {
            this.sensorValue = sensorValue;
            this.sensorZone = sensorZone;
            this.prevZone = sensorZone;
            this.triggerEnabled = triggerEnabled;
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
    private final TrcValueSource<Double> valueSource;
    private final TriggerState triggerState;
    private final CallbackContext callbackContext;
    private final TrcTaskMgr.TaskObject triggerTaskObj;
    private double[] thresholds;
    private TriggerMode triggerMode = null;
    private TrcEvent triggerEvent = null;
    private TrcEvent.Callback triggerCallback = null;
    private Thread callbackThread = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param valueSource specifies the interface that implements the value source.
     * @param dataPoints specifies an array of trigger points or an array of thresholds if dataIsTrigger is true.
     * @param dataIsTrigger specifies true if dataPoints specifies an array of trigger points, false if it is an
     *                      array of thresholds. Trigger points will be converted to threshold points.
     */
    public TrcTriggerThresholdZones(
        String instanceName, TrcValueSource<Double> valueSource, double[] dataPoints, boolean dataIsTrigger)
    {
        if (valueSource == null)
        {
            throw new IllegalArgumentException("ValueSource cannot be null.");
        }

        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.valueSource = valueSource;
        if (dataIsTrigger)
        {
            setTriggerPoints(dataPoints);
        }
        else
        {
            setThresholds(dataPoints);
        }

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
     * This method arms/disarms the trigger. It enables/disables the task that monitors the sensor value.
     *
     * @param enabled specifies true to enable, false to disable.
     * @param event specifies the event to signal when the trigger state changed, ignored if enabled is false.
     */
    private void setEnabled(boolean enabled, TrcEvent event)
    {
        synchronized (triggerState)
        {
            if (enabled)
            {
                event.clear();
                triggerEvent = event;
                triggerState.sensorValue = getSensorValue();
                triggerState.sensorZone = getValueZone(triggerState.sensorValue);
                triggerState.prevZone = triggerState.sensorZone;
                triggerTaskObj.registerTask(TrcTaskMgr.TaskType.PRE_PERIODIC_TASK);
            }
            else
            {
                triggerTaskObj.unregisterTask();
                triggerEvent. cancel();
                triggerEvent = null;
            }
            triggerState.triggerEnabled = enabled;
            tracer.traceDebug(instanceName, "enabled=" + enabled + " (state=" + triggerState + ")");
        }
    }   //setEnabled

    /**
     * This method arms the trigger. It enables the task that monitors the sensor value.
     *
     * @param triggerMode specifies the trigger direction that will signal the event. TriggerMode.OnActive will
     *        trigger only when crossing a lower zone to a higher zone. TriggerMode.OnInactive will trigger only when
     *        crossing from a higher zone to a lower zone. TriggerMode.OnBoth will trigger on both directions.
     * @param event specifies the event to signal when the trigger state changed.
     */
    @Override
    public void enableTrigger(TriggerMode triggerMode, TrcEvent event)
    {
        this.triggerMode = triggerMode;
        this.triggerCallback = null;
        this.callbackThread = null;
        setEnabled(true, event);
    }   //enableTrigger

    /**
     * This method arms the trigger. It enables the task that monitors the sensor value.
     *
     * @param triggerMode specifies the trigger direction that will trigger a callback. TriggerMode.OnActive will
     *        trigger only when crossing a lower zone to a higher zone. TriggerMode.OnInactive will trigger only when
     *        crossing from a higher zone to a lower zone. TriggerMode.OnBoth will trigger on both directions.
     * @param callback specifies the callback handler to notify when the trigger state changed.
     */
    @Override
    public void enableTrigger(TriggerMode triggerMode, TrcEvent.Callback callback)
    {
        this.triggerMode = triggerMode;
        this.triggerCallback = callback;
        this.callbackThread = Thread.currentThread();
        setEnabled(true, new TrcEvent(instanceName + ".triggerEvent"));
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
                this.triggerMode = null;
                this.triggerCallback = null;
                this.callbackThread = null;
                setEnabled(false, null);
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
     * This method reads the current sensor value. It may return null if it failed to read the sensor.
     *
     * @return current sensor value.
     */
    @Override
    public double getSensorValue()
    {
        return valueSource.getValue();
    }   //getSensorValue

    /**
     * This method reads the current digital sensor state (not supported).
     *
     * @return current sensor state.
     */
    @Override
    public boolean getSensorState()
    {
        throw new UnsupportedOperationException("Analog sensor does not support digital state.");
    }   //getSensorState

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

        if (value < thresholds[0])
        {
            zone = 0;
        }
        else
        {
            for (int i = 0; i < thresholds.length - 1; i++)
            {
                if (value >= thresholds[i] && value < thresholds[i + 1])
                {
                    zone = i + 1;
                    break;
                }
            }

            if (zone == -1)
            {
                zone = thresholds.length;
            }
        }
        tracer.traceDebug(instanceName, "value=%f, zone=%d", value, zone);

        return zone;
    }   //getValueZone

    /**
     * This method creates and threshold array and calculates all the threshold values. A threshold value is the
     * average of two adjacent trigger points.
     *
     * @param triggerPoints specifies the array of trigger points.
     */
    public void setTriggerPoints(double[] triggerPoints)
    {
        if (triggerPoints == null || triggerPoints.length < 2)
        {
            throw new IllegalArgumentException("triggerPoints cannot be null and must have at least 2 points.");
        }

        thresholds = new double[triggerPoints.length - 1];
        for (int i = 0; i < thresholds.length; i++)
        {
            thresholds[i] = (triggerPoints[i] + triggerPoints[i + 1])/2.0;
        }
        tracer.traceDebug(
            instanceName,
            "triggerPts=" + Arrays.toString(triggerPoints) +
            ", thresholds=" + Arrays.toString(thresholds));
    }   //setTriggerPoints

    /**
     * This method stores the threshold array.
     *
     * @param thresholds specifies the array of thresholds.
     */
    public void setThresholds(double[] thresholds)
    {
        if (thresholds == null || thresholds.length == 0)
        {
            throw new IllegalArgumentException("thresholds cannot be null nor empty.");
        }

        this.thresholds = Arrays.copyOf(thresholds, thresholds.length);
        tracer.traceDebug(instanceName, "thresholds=" + Arrays.toString(thresholds));
    }   //setThresholds

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
                if (triggerMode == TriggerMode.OnBoth ||
                    triggerMode == TriggerMode.OnActive && prevZone < currZone ||
                    triggerMode == TriggerMode.OnInactive && prevZone > currZone)
                {
                    triggered = true;
                }
            }
            triggerState.sensorValue = currValue;
        }

        if (triggered)
        {
            tracer.traceDebug(instanceName, "Crossing zones %d->%d (value=%f)", prevZone, currZone, currValue);
            if (triggerCallback != null)
            {
                synchronized (callbackContext)
                {
                    callbackContext.sensorValue = currValue;
                    callbackContext.prevZone = prevZone;
                    callbackContext.currZone = currZone;
                }
                triggerEvent.setCallback(callbackThread, triggerCallback, callbackContext);
            }
            triggerEvent.signal();
        }
    }   //triggerTask

}   //class TrcTriggerThresholdZones
