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

package trclib.sensor;

import java.util.Arrays;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;

import trclib.dataprocessor.TrcDataBuffer;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements a Threshold Range Trigger. It monitors the value source against the lower and upper
 * threshold values. If the value stays within the lower and upper thresholds for at least the given settling
 * period, the the trigger state is set to active and the trigger callback is called or an event is signaled.
 * If the value exits the threshold range, the trigger state is set to inactive and the trigger callback is
 * also called or an event signaled.
 */
public class TrcTriggerThresholdRange implements TrcTrigger
{
    /**
     * This class contains trigger parameters.
     */
    public static class TriggerParams
    {
        public double lowThreshold = 0.0;
        public double highThreshold = 0.0;
        public double settlingPeriod = 0.0;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param lowThreshold specifies the low threshold value for the trigger.
         * @param highThreshold specifies the high threshold value for the trigger.
         * @param settingPeriod specifies the minimum period in seconds sensor value must stay within trigger
         *        thresholds to be considered triggered.
         */
        public TriggerParams(double lowThreshold, double highThreshold, double settingPeriod)
        {
            this.lowThreshold = lowThreshold;
            this.highThreshold = highThreshold;
            this.settlingPeriod = settingPeriod;
        }   //TriggerParams

        @Override
        public String toString()
        {
            return "(lowThreshold=" + lowThreshold +
                   ",highThreshold=" + highThreshold +
                   ",settingPeriod=" + settlingPeriod + ")";
        }   //toString
    }   //TriggerParams

    private static final int DEF_CACHE_SIZE = 10;
    /**
     * This class encapsulates the trigger state. Access to this object must be thread safe (i.e. needs to be
     * synchronized).
     */
    private static class TriggerState
    {
        TriggerParams triggerParams = null;
        volatile boolean triggerActive = false;
        volatile double startTime = 0.0;
        volatile boolean triggerEnabled = false;
        TrcDataBuffer cachedData = null;
        TriggerMode triggerMode;
        TrcEvent triggerEvent;
        TrcEvent.Callback triggerCallback;
        Thread callbackThread;

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "(triggerParams=%s,active=%s,startTime=%.6f,enabled=%s,data=%s)",
                triggerParams, triggerActive, startTime, triggerEnabled,
                cachedData != null? Arrays.toString(cachedData.getBufferedData()): "null");
        }   //toString

    }   //class TriggerState

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final DoubleSupplier valueSource;
    private final TrcTimer timer;
    private final TriggerState triggerState;
    private final AtomicBoolean callbackContext;
    private final TrcTaskMgr.TaskObject triggerTaskObj;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param valueSource specifies the interface that implements the value source.
     */
    public TrcTriggerThresholdRange(String instanceName, DoubleSupplier valueSource)
    {
        if (valueSource == null)
        {
            throw new IllegalArgumentException("ValueSource cannot be null.");
        }

        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.valueSource = valueSource;
        timer = new TrcTimer(instanceName);
        triggerState = new TriggerState();
        callbackContext = new AtomicBoolean();
        triggerTaskObj = TrcTaskMgr.createTask(instanceName + ".triggerTask", this::triggerTask);
    }   //TrcTriggerThresholdRange

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
     * This method sets the trigger notify parameters.
     *
     * @param mode specifies the trigger mode.
     * @param event specifies the event to signal when trigger occurs.
     */
    private void setTriggerNotifyParams(TriggerMode mode, TrcEvent event)
    {
        synchronized (triggerState)
        {
            triggerState.triggerMode = mode;
            triggerState.triggerEvent = event;
            triggerState.triggerCallback = null;
            triggerState.callbackThread = null;
        }
    }   //setTriggerNotifyParams

    /**
     * This method sets the trigger notify parameters.
     *
     * @param mode specifies the trigger mode.
     * @param callback specifies the callback method to call when trigger occurs.
     */
    private void setTriggerNotifyParams(TriggerMode mode, TrcEvent.Callback callback)
    {
        synchronized (triggerState)
        {
            triggerState.triggerMode = mode;
            triggerState.triggerEvent = new TrcEvent(instanceName + ".triggerEvent");
            triggerState.triggerCallback = callback;
            triggerState.callbackThread = Thread.currentThread();
        }
    }   //setTriggerNotifyParams

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
                if (triggerState.triggerParams == null)
                {
                    throw new RuntimeException("Must call setTrigger first before enabling the trigger.");
                }

                triggerState.triggerEvent.clear();
                triggerState.startTime = TrcTimer.getCurrentTime();
                triggerState.cachedData.clear();
                triggerTaskObj.registerTask(TrcTaskMgr.TaskType.PRE_PERIODIC_TASK);
            }
            else
            {
                triggerTaskObj.unregisterTask();
                triggerState.triggerEvent.cancel();
                triggerState.triggerEvent = null;
            }
            triggerState.triggerActive = false;
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
                setTriggerNotifyParams(triggerMode, event);
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
                setTriggerNotifyParams(triggerMode, callback);
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
     * This method reads the current trigger state. It returns triggered if the analog sensor value is within the
     * trigger zone.
     *
     * @return current trigger state.
     */
    @Override
    public boolean getTriggerState()
    {
        return triggerState.triggerActive;
    }   //getTriggerState

    /**
     * This method sets the low/high threshold values within which the sensor reading must stay for at least the
     * settling period for it to trigger the notification.
     *
     * @param triggerParams specifies the trigger threshold range parameters.
     * @param maxCachedSize specifies the max number of of cached values.
     */
    public void setTrigger(TriggerParams triggerParams, int maxCachedSize)
    {
        tracer.traceDebug(instanceName, "triggerParams=%s, maxCachedSize=%d", triggerParams, maxCachedSize);
        synchronized (triggerState)
        {
            triggerState.triggerParams = triggerParams;
            triggerState.cachedData = new TrcDataBuffer(instanceName, maxCachedSize);
        }
    }   //setTrigger

    /**
     * This method sets the low/high threshold values within which the sensor reading must stay for at least the
     * settling period for it to trigger the notification.
     *
     * @param triggerParams specifies the trigger threshold range parameters.
     */
    public void setTrigger(TriggerParams triggerParams)
    {
        setTrigger(triggerParams, DEF_CACHE_SIZE);
    }   //setTrigger

    /**
     * This method sets the low/high threshold values within which the sensor reading must stay for at least the
     * settling period for it to trigger the notification.
     *
     * @param lowThreshold specifies the low threshold value for the trigger.
     * @param highThreshold specifies the high threshold value for the trigger.
     * @param settlingPeriod specifies the period in seconds the sensor value must stay within threshold range for it
     *        to trigger.
     */
    public void setTrigger(double lowThreshold, double highThreshold, double settlingPeriod)
    {
        synchronized (triggerState)
        {
            if (triggerState.triggerParams != null)
            {
                triggerState.triggerParams.lowThreshold = lowThreshold;
                triggerState.triggerParams.highThreshold = highThreshold;
            }
            else
            {
                tracer.traceErr(instanceName, "Initial trigger params were not set up.");
            }
        }
    }   //setTrigger

    /**
     * This method returns an array of the data recorded during the trigger settling period.
     *
     * @return array of trigger settling data, null if no data recorded.
     */
    public Double[] getTriggerSettlingData()
    {
        Double[] data = null;

        synchronized (triggerState)
        {
            if (triggerState.cachedData != null)
            {
                data = triggerState.cachedData.getBufferedData();
            }
        }

        return data;
    }   //getTriggerSettlingData

    /**
     * This method returns the minimum sensor value recorded in the cache. Cache only records values within thresholds.
     *
     * @return minimum value in the cache, null if cache was empty.
     */
    public Double getMinimumValue()
    {
        Double minValue = null;

        synchronized (triggerState)
        {
            if (triggerState.cachedData != null)
            {
                minValue = triggerState.cachedData.getMinimumValue();
            }
        }

        return minValue;
    }   //getMinimumValue

    /**
     * This method returns the maximum sensor value recorded in the cache. Cache only records values within thresholds.
     *
     * @return maximum value in the cache, null if cache was empty.
     */
    public Double getMaximumValue()
    {
        Double maxValue = null;

        synchronized (triggerState)
        {
            if (triggerState.cachedData != null)
            {
                maxValue = triggerState.cachedData.getMaximumValue();
            }
        }

        return maxValue;
    }   //getMaximumValue

    /**
     * This method calculates the average sensor value recorded in the cache.
     *
     * @return average value calculated.
     */
    public Double getAverageValue()
    {
        Double avgValue = null;

        synchronized (triggerState)
        {
            if (triggerState.cachedData != null)
            {
                avgValue = triggerState.cachedData.getAverageValue();
            }
        }

        return avgValue;
    }   //getAverageValue

    /**
     * This method is called periodically to check if the sensor value is within the lower and upper threshold range.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void triggerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        double currTime = TrcTimer.getCurrentTime();
        double currValue = getSensorValue();
        boolean triggered = false;
        boolean active;
        TrcEvent.Callback callback = null;
        TrcEvent triggerEvent = null;
        Thread callbackThread = null;

        synchronized (triggerState)
        {
            if (currValue < triggerState.triggerParams.lowThreshold ||
                currValue > triggerState.triggerParams.highThreshold)
            {
                // Outside of threshold range.
                if (triggerState.triggerActive)
                {
                    // Only fires a trigger event when the threshold range is first exited (i.e. edge event).
                    triggerState.triggerActive = false;
                    if (triggerState.triggerMode == TriggerMode.OnBoth ||
                        triggerState.triggerMode == TriggerMode.OnInactive)
                    {
                        triggered = true;
                        callback = triggerState.triggerCallback;
                        triggerEvent = triggerState.triggerEvent;
                        callbackThread = triggerState.callbackThread;
                    }
                }
                triggerState.startTime = currTime;
                triggerState.cachedData.clear();
            }
            else if (currTime >= triggerState.startTime + triggerState.triggerParams.settlingPeriod)
            {
                // Inside of threshold range and past settling period.
                if (!triggerState.triggerActive)
                {
                    // Only fires a trigger event when the threshold range is first entered and stayed for settling
                    // period (i.e. edge event).
                    triggerState.cachedData.addValue(currValue);
                    triggerState.triggerActive = true;
                    if (triggerState.triggerMode == TriggerMode.OnBoth ||
                        triggerState.triggerMode == TriggerMode.OnActive)
                    {
                        triggered = true;
                        callback = triggerState.triggerCallback;
                        triggerEvent = triggerState.triggerEvent;
                        callbackThread = triggerState.callbackThread;
                    }
                }
            }
            else
            {
                triggerState.cachedData.addValue(currValue);
            }
            active = triggerState.triggerActive;
        }
        // Do not hold the lock while doing callback.
        if (triggered)
        {
            tracer.traceDebug(instanceName, "Triggered (state=" + active + ")");
            if (callback != null)
            {
                callbackContext.set(active);
                triggerEvent.setCallback(callbackThread, callback, callbackContext);
            }
            triggerEvent.signal();
        }
    }   //triggerTask

}   //class TrcTriggerThresholdRange
