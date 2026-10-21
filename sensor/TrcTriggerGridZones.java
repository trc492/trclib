/*
 * Copyright (c) 2026 Titan Robotics Club (http://www.titanrobotics.com)
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

/**
 * This class implements a 2-dimensional Grid Zones Trigger. It monitors the grid position source against a
 * 2-dimensional array of threshold points forming a grid. If the source grid position crosses the grid line
 * in either directions, it signals an event or notifies a callback so that an action could be performed.
 */
public class TrcTriggerGridZones implements TrcTrigger
{
    public static class GridTriggerInfo
    {
        public TrcTriggerThresholdZones.TriggerInfo xTriggerInfo = null;
        public TrcTriggerThresholdZones.TriggerInfo yTriggerInfo = null;

        public GridTriggerInfo(
            TrcTriggerThresholdZones.TriggerInfo xTriggerInfo, TrcTriggerThresholdZones.TriggerInfo yTriggerInfo)
        {
            this.xTriggerInfo = xTriggerInfo;
            this.yTriggerInfo = yTriggerInfo;
        }   //GridTriggerInfo

        @Override
        public GridTriggerInfo clone()
        {
            return new GridTriggerInfo(xTriggerInfo.clone(), yTriggerInfo.clone());
        }   //clone
    }   //class GridTriggerInfo

    /**
     * This class encapsulates the trigger state. Access to this object must be thread safe (i.e. needs to be
     * synchronized).
     */
    private static class TriggerState
    {
        GridTriggerInfo gridTriggerInfo = new GridTriggerInfo(null, null);
        boolean triggerEnabled = false;
        TrcEvent triggerEvent = null;
        TrcEvent.Callback triggerCallback = null;

        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "(xTriggerInfo=%s,yTriggerInfo=%s,enabled=%s,event=%s,callback=%s)",
                gridTriggerInfo.xTriggerInfo, gridTriggerInfo.yTriggerInfo, triggerEnabled, triggerEvent,
                triggerCallback != null);
        }   //toString

    }   //class TriggerState

    private final String instanceName;
    private final TrcTriggerThresholdZones gridXZoneTrigger;
    private final TrcTriggerThresholdZones gridYZoneTrigger;
    public final TrcDbgTrace tracer;
    private final TriggerState triggerState;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gridXSource specifies the method to call to get the grid X value.
     * @param gridYSource specifies the method to call to get the grid Y value.
     * @param gridXThresholdPoints specifies an array of grid X threshold points for the trigger.
     * @param gridYThresholdPoints specifies an array of grid Y threshold points for the trigger.
     */
    public TrcTriggerGridZones(
        String instanceName, DoubleSupplier gridXSource, DoubleSupplier gridYSource,
        double[] gridXThresholdPoints, double[] gridYThresholdPoints)
    {
        if (gridXSource == null || gridYSource == null)
        {
            throw new IllegalArgumentException("gridXSource or gridYSource cannot be null.");
        }

        if (gridXThresholdPoints == null || gridYThresholdPoints == null)
        {
            throw new IllegalArgumentException("gridXThresholdPoints or gridYThresholdPoints cannot be null.");
        }

        this.instanceName = instanceName;
        this.gridXZoneTrigger = new TrcTriggerThresholdZones(
            instanceName + ".gridX", gridXSource, gridXThresholdPoints);
        this.gridYZoneTrigger = new TrcTriggerThresholdZones(
            instanceName + ".gridY", gridYSource, gridYThresholdPoints);
        this.tracer = gridXZoneTrigger.tracer;
        this.triggerState = new TriggerState();
    }   //TrcTriggerThresholdZones

    /**
     * This method returns the instance name and its state.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    //
    // Implements TrcTrigger interface.
    //

    /**
     * This method arms the trigger. It enables both X and Y grid zone triggers.
     *
     * @param triggerDelay specifies the delay in seconds before enabling trigger, null if none.
     * @param triggerMode specifies the trigger direction that will signal the event. TriggerMode.OnActive will
     *        trigger only when crossing a lower zone to a higher zone. TriggerMode.OnInactive will trigger only when
     *        crossing from a higher zone to a lower zone. TriggerMode.OnBoth will trigger on both directions.
     * @param event specifies the event to signal when the trigger state changed.
     * @param callback specifies the method to call back when a trigger event occurs.
     */
    private void enableTrigger(
        Double triggerDelay, TriggerMode triggerMode, TrcEvent event, TrcEvent.Callback callback)
    {
        synchronized (triggerState)
        {
            if (!triggerState.triggerEnabled)
            {
                triggerState.triggerEvent = event;
                triggerState.triggerCallback = callback;
                if (event != null)
                {
                    event.clear();
                }

                gridXZoneTrigger.enableTrigger(triggerDelay, triggerMode, this::gridXTriggerCallback);
                gridYZoneTrigger.enableTrigger(triggerDelay, triggerMode, this::gridYTriggerCallback);
                triggerState.gridTriggerInfo.xTriggerInfo = gridXZoneTrigger.getTriggerInfo();
                triggerState.gridTriggerInfo.yTriggerInfo = gridYZoneTrigger.getTriggerInfo();
                triggerState.triggerEnabled = true;
                tracer.traceDebug(instanceName, "EnableTrigger: State=(" + triggerState + ")");
            }
        }
    }   //enableTrigger

    /**
     * This method arms the trigger. It enables both X and Y grid zone triggers.
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
        enableTrigger(triggerDelay, triggerMode, event, null);
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
        enableTrigger(triggerDelay, triggerMode, null, callback);
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
                gridXZoneTrigger.disableTrigger();
                gridYZoneTrigger.disableTrigger();
                triggerState.triggerEvent = null;
                triggerState.triggerCallback = null;
                triggerState.triggerEnabled = false;
                tracer.traceDebug(instanceName, "DisableTrigger.");
            }
        }
    }   //disableTrigger

    /**
     * This method is called when either Grid X or Grid Y trigger has occurred.
     * Note: This method assumes the caller has taken the triggerState lock.
     *
     * @param triggerInfo specifies the info of the trigger that occurs.
     * @param canceled specifies true if the trigger was canceled, false otherwise.
     * @param isGridXTrigger specifies true if the trigger is from GridX, false if from GridY.
     */
    private void gridTriggerCallback(
        TrcTriggerThresholdZones.TriggerInfo triggerInfo, boolean canceled, boolean isGridXTrigger)
    {
        if (!canceled)
        {
            TrcEvent event;
            TrcEvent.Callback callback;
            GridTriggerInfo gridTriggerInfo = null;

            synchronized (triggerState)
            {
                if (isGridXTrigger)
                {
                    triggerState.gridTriggerInfo.xTriggerInfo = triggerInfo;
                }
                else
                {
                    triggerState.gridTriggerInfo.yTriggerInfo = triggerInfo;
                }
                event = triggerState.triggerEvent;
                callback = triggerState.triggerCallback;
                if (callback != null)
                {
                    gridTriggerInfo = new GridTriggerInfo(
                        triggerState.gridTriggerInfo.xTriggerInfo.clone(),
                        triggerState.gridTriggerInfo.yTriggerInfo.clone());
                }
            }

            if (event != null)
            {
                event.signal();
            }

            if (callback != null)
            {
                // It's safe to do the callback directly because gridTriggerCallback is called from the caller's
                // thread.
                callback.notify(gridTriggerInfo, false);
            }
        }
    }   //gridTriggerCallback

    private void gridXTriggerCallback(Object context, boolean canceled)
    {
        gridTriggerCallback((TrcTriggerThresholdZones.TriggerInfo) context, canceled, true);
    }   //gridXTriggerCallback

    private void gridYTriggerCallback(Object context, boolean canceled)
    {
        gridTriggerCallback((TrcTriggerThresholdZones.TriggerInfo) context, canceled, false);
    }   //gridYTriggerCallback

    public GridTriggerInfo getGridTriggerInfo()
    {
        synchronized (triggerState)
        {
            return triggerState.gridTriggerInfo.clone();
        }
    }   //getGridTriggerInfo

    /**
     * This method checks if the trigger task is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    @Override
    public boolean isEnabled()
    {
        synchronized (triggerState)
        {
            return triggerState.triggerEnabled;
        }
    }   //isEnabled

    /**
     * This method reads the current sensor value. TrcTriggerGridZones does not support getSensorValue and
     * will throw an exception.
     *
     * @throws UnsupportedOperationException for unsupported operation.
     */
    @Override
    public double getSensorValue()
    {
        throw new UnsupportedOperationException("TrcTriggerGridZones does not support getSensorValue.");
    }   //getSensorValue

    /**
     * This method reads the current trigger state. TrcTriggerGridZones does not support getTriggerState and
     * will throw an exception.
     *
     * @throws UnsupportedOperationException for unsupported operation.
     */
    @Override
    public boolean getSensorState()
    {
        throw new UnsupportedOperationException("TrcTriggerGridZones does not support sensor state.");
    }   //getSensorState

}   //class TrcTriggerGridZones
