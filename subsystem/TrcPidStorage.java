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

/**
 * This class implements a platform independent PID Controlled Storage Subsystem. It consists of a motor or a
 * continuous rotation servo that transports the game elements in the storage from the entrance to the exit.
 * Optionally, it may have entry and exit sensors to detect the game element entering or exiting the Storage and
 * allows callback actions such as advancing the Storage to the next position.
 */
public class TrcPidStorage implements TrcExclusiveSubsystem
{
    /**
     * This class contains storage parameters.
     */
    public static class StorageParams
    {
        private double objectDistance = 0.0;
        private double movePower = 1.0;
        private int maxCapacity = 1;

        /**
         * This method returns the string format of the PID storage parameters.
         *
         * @return string format of the parameters.
         */
        @Override
        public String toString()
        {
            return "(objDistance=" + objectDistance +
                   ",movePower=" + movePower +
                   ",maxCap=" + maxCapacity + ")";
        }   //toString

        /**
         * This method sets the distance between the adjacent objects in the storage.
         *
         * @param objectDistance specifies object distance.
         * @return this object for chaining.
         */
        public StorageParams setObjectDistance(double objectDistance)
        {
            this.objectDistance = objectDistance;
            return this;
        }   //setObjectDistance

        /**
         * This method sets the motor power used to move the objects inside the storage.
         *
         * @param movePower specifies motor power used to move objects.
         * @return this object for chaining.
         */
        public StorageParams setMovePower(double movePower)
        {
            this.movePower = movePower;
            return this;
        }   //setMovePower

        /**
         * This method sets the maximum number of objects the storage can hold.
         *
         * @param maxCapacity specifies the maximum number of objects the storage can hold.
         * @return this object for chaining.
         */
        public StorageParams setMaxCapacity(int maxCapacity)
        {
            this.maxCapacity = maxCapacity;
            return this;
        }   //setMaxCapacity

    }   //class StorageParams

    /**
     * This class contains all the parameters of a Storage Trigger. The parameters specify the action it will take
     * when the trigger occurs. The trigger can optionally provide a notification callback.
     */
    public static class TriggerParams
    {
        private final TrcTrigger trigger;
        private final boolean advanceOnTrigger;
        private final TrcEvent.Callback triggerCallback;
        private final Object callbackContext;

        public TriggerParams(
            TrcTrigger trigger, boolean advanceOnTrigger, TrcEvent.Callback callback, Object callbackContext)
        {
            this.trigger = trigger;
            this.advanceOnTrigger = advanceOnTrigger;
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
                   ", advanceOnTrigger=" + advanceOnTrigger +
                   ", triggerCallback=" + (triggerCallback != null) +
                   ", callbackContext=" + (callbackContext != null);
        }   //toString

    }   //class TriggerParams

    public final TrcDbgTrace tracer;
    protected final String instanceName;
    public final TrcMotor motor;
    private final StorageParams storageParams;
    private final TriggerParams entryTriggerParams;
    private final TriggerParams exitTriggerParams;
    private int numObjects = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor object.
     * @param storageParams specifies the storage parameters.
     * @param entryTriggerParams specifies the entry sensor trigger parameters, can be null if no entry sensor.
     * @param exitTriggerParams specifies the exit sensor trigger parameters, can be null if no exit sensor.
     */
    public TrcPidStorage(
        String instanceName, TrcMotor motor, StorageParams storageParams, TriggerParams entryTriggerParams,
        TriggerParams exitTriggerParams)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.motor = motor;
        this.storageParams = storageParams;
        this.entryTriggerParams = entryTriggerParams;
        this.exitTriggerParams = exitTriggerParams;
    }   //TrcPidStorage

    /**
     * This method enables/disables the entry trigger.
     *
     * @param enabled specifies true to enable trigger, false to disable.
     */
    public void setEntryTriggerEnabled(boolean enabled)
    {
        tracer.traceInfo(instanceName, "setEntryTriggerEnabled(enabled=" + enabled + ")");
        if (entryTriggerParams != null)
        {
            if (enabled)
            {
                entryTriggerParams.trigger.enableTrigger(TrcTrigger.TriggerMode.OnBoth, this::onEntryTrigger);
            }
            else
            {
                entryTriggerParams.trigger.disableTrigger();
            }
        }
    }   //setEntryTriggerEnabled

    /**
     * This method enables/disables the exit trigger.
     *
     * @param enabled specifies true to enable trigger, false to disable.
     */
    public void setExitTriggerEnabled(boolean enabled)
    {
        tracer.traceInfo(instanceName, "setExitTriggerEnabled(enabled=" + enabled + ")");
        if (exitTriggerParams != null)
        {
            if (enabled)
            {
                exitTriggerParams.trigger.enableTrigger(TrcTrigger.TriggerMode.OnBoth, this::onExitTrigger);
            }
            else
            {
                exitTriggerParams.trigger.disableTrigger();
            }
        }
    }   //setExitTriggerEnabled

    /**
     * This method cancels the operation which is stopping the motor.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the storage.
     */
    public void cancel(String owner)
    {
        tracer.traceInfo(instanceName, "cancel(owner=" + owner + ")");
        if (validateOwnership(owner))
        {
            motor.cancel();
        }
    }   //cancel

    /**
     * This method cancels the operation which is stopping the motor.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    public void zeroCalibrate(String owner, double calPower, TrcEvent event)
    {
        tracer.traceInfo(
            instanceName, "zeroCalibrate(owner=" + owner + ", calPower=" + calPower + ", event=" + event + ")");
        motor.zeroCalibrate(owner, calPower, event);
    }   //zeroCalibrate

    /**
     * This method returns the entry sensor trigger if there is one. It returns null if there isn't one.
     *
     * @return entry sensor trigger, or null if there isn't one.
     */
    public TrcTrigger getEntryTrigger()
    {
        return entryTriggerParams.trigger;
    }   //getEntryTrigger

    /**
     * This method returns the exit sensor trigger if there is one. It returns null if there isn't one.
     *
     * @return exit sensor trigger, or null if there isn't one.
     */
    public TrcTrigger getExitTrigger()
    {
        return exitTriggerParams.trigger;
    }   //getExitTrigger

    /**
     * This method returns the number of objects in the storage.
     * Note: Storage can only keep track of the number of objects if it has entry and exit sensors. Otherwise,
     * the number of objects returned will not be correct.
     *
     * @return number of objects in the storage.
     */
    public int getNumObjects()
    {
        return numObjects;
    }   //getNumObjects

    /**
     * This method sets the number of preloaded objects in the storage.
     *
     * @param num specifies the number of preloaded objects in the storage.
     */
    public void setPreloadedObjects(int num)
    {
        this.numObjects = num;
    }   //setPreloadedObjects

    /**
     * This method returns the sensor state read from the digital sensor or digital source.
     *
     * @return digital sensor state.
     */
    public boolean isEntrySensorActive()
    {
        return entryTriggerParams != null && entryTriggerParams.trigger.getTriggerState();
    }   //isEntrySensorActive

    /**
     * This method returns the sensor state read from the digital sensor or digital source.
     *
     * @return digital sensor state.
     */
    public boolean isExitSensorActive()
    {
        return exitTriggerParams != null && exitTriggerParams.trigger.getTriggerState();
    }   //isExitSensorActive

    /**
     * This method advances or backs up the storage by the number of object units.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the storage.
     * @param units specifies the number of object units to advance or negative number to back up.
     * @param event specifies the event to signal when done, can be null if not provided.
     */
    public void move(String owner, int units, TrcEvent event)
    {
        if (validateOwnership(owner))
        {
            if (event != null)
            {
                event.clear();
            }
            motor.setPosition(0.0, units*storageParams.objectDistance, false, storageParams.movePower, event);
        }
    }   //move

    /**
     * This method advances or backs up the storage by the number of object units.
     *
     * @param units specifies the number of object units to advance or negative number to back up.
     * @param event specifies the event to signal when done, can be null if not provided.
     */
    public void move(int units, TrcEvent event)
    {
        move(null, units, event);
    }   //move

    /**
     * This method advances or backs up the storage by the number of object units.
     *
     * @param units specifies the number of object units to advance or negative number to back up.
     */
    public void move(int units)
    {
        move(null, units, null);
    }   //move

    /**
     * This method advances the storage by the one object unit.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the storage.
     */
    public void advance(String owner)
    {
        move(owner, 1, null);
    }   //advance

    /**
     * This method advances the storage by the one object unit.
     */
    public void advance()
    {
        move(null, 1, null);
    }   //advance

    /**
     * This method backs up the storage by the one object unit.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the storage.
     */
    public void backup(String owner)
    {
        move(owner, -1, null);
    }   //backup

    /**
     * This method backs up the storage by the one object unit.
     */
    public void backup()
    {
        move(null, -1, null);
    }   //backup

    /**
     * This method is called when the entry sensor is triggered.
     *
     * @param context specifies true if an object has activated the sensor, false if the object has deactivated it.
     * @param canceled specifies true if trigger was disabled.
     */
    private void onEntryTrigger(Object context, boolean canceled)
    {
        if (!canceled)
        {
            boolean active = ((AtomicBoolean) context).get();

            if (active)
            {
                numObjects++;
                if (entryTriggerParams.advanceOnTrigger)
                {
                    advance();
                }
            }
            tracer.traceInfo(
                instanceName,
                "onEntryTrigger(active=" + active +
                ", numObj=" + numObjects +
                ", advOnTrigger=" + entryTriggerParams.advanceOnTrigger);

            if (entryTriggerParams.triggerCallback != null)
            {
                TrcEvent callbackEvent = new TrcEvent(instanceName + ".entryCallbackEvent");
                callbackEvent.setCallback(entryTriggerParams.triggerCallback, entryTriggerParams.callbackContext);
                callbackEvent.signal();
            }
        }
        else
        {
            tracer.traceInfo(instanceName, "Cancel entry trigger.");
        }
    }   //onEntryTrigger

    /**
     * This method is called when the exit sensor is triggered.
     *
     * @param context specifies true if an object has activated the sensor, false if the object has deactivated it.
     * @param canceled specifies true if trigger was disabled.
     */
    private void onExitTrigger(Object context, boolean canceled)
    {
        if (!canceled)
        {
            boolean active = ((AtomicBoolean) context).get();

            if (!active)
            {
                numObjects--;
                if (exitTriggerParams.advanceOnTrigger)
                {
                    advance();
                }
            }
            tracer.traceInfo(
                instanceName,
                "onExitTrigger(active=" + active +
                ", numObj=" + numObjects +
                ", advOnTrigger=" + exitTriggerParams.advanceOnTrigger);

            if (exitTriggerParams.triggerCallback != null)
            {
                TrcEvent callbackEvent = new TrcEvent(instanceName + ".exitCallbackEvent");
                callbackEvent.setCallback(exitTriggerParams.triggerCallback, exitTriggerParams.callbackContext);
                callbackEvent.signal();
            }
        }
        else
        {
            tracer.traceInfo(instanceName, "Cancel exit trigger.");
        }
    }   //onExitTrigger

}   //class TrcPidStorage
