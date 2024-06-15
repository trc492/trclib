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

package trclib.archive;

import java.util.concurrent.atomic.AtomicBoolean;

import trclib.dataprocessor.TrcTrigger.TriggerMode;
import trclib.dataprocessor.TrcTriggerDigitalInput;
import trclib.sensor.TrcDigitalInput;

/**
 * This class implements a platform independent conveyor subsystem. It contains a motor and optionally an entry
 * and an exit sensor that detects if the an object has entered or exited the conveyor. It also supports exclusive
 * subsystem access by implementing TrcExclusiveSubsystem. This enables the conveyor subsystem to be aware of multiple
 * callers' access to the subsystem. While one caller starts the conveyor for an operation, nobody can access it until
 * the previous caller is done with the operation.
 */
public class TrcPidConveyor implements TrcExclusiveSubsystem
{
    /**
     * This class contains all the parameters related to the conveyor.
     */
    public static class Parameters
    {
        public double objectDistance = 0.0;
        public double movePower = 1.0;
        public int maxCapacity = 1;

        /**
         * This method returns the string format of the PID conveyor parameters.
         *
         * @return string format of the parameters.
         */
        @Override
        public String toString()
        {
            return "(objDistance=" + objectDistance + ",movePower=" + movePower + ",maxCap=" + maxCapacity + ")";
        }   //toString

        /**
         * This method sets the distance between objects inside the converyor.
         *
         * @param distance specifies the distance between objects in the conveyor.
         * @return this parameter object.
         */
        public Parameters setObjectDistance(double distance)
        {
            this.objectDistance = distance;
            return this;
        }   //setObjectDistance

        /**
         * This method sets the conveyor move power.
         *
         * @param power specifies the motor power to move the object in the conveyor.
         * @return this parameter object.
         */
        public Parameters setMovePower(double power)
        {
            this.movePower = power;
            return this;
        }   //setMovePower

        /**
         * This method sets the maximum number of objects the conveyor can hold.
         *
         * @param capacity specifies maximum number of objects the conveyor can hold.
         * @return this parameter object.
         */
        public Parameters setMaxCapacity(int capacity)
        {
            this.maxCapacity = capacity;
            return this;
        }   //setMaxCapacity

    }   //class Parameters

    public final TrcMotor motor;
    private final TrcDigitalInput entrySensor;
    private final TrcDigitalInput exitSensor;
    private final Parameters params;
    private final TrcTriggerDigitalInput entryTrigger;
    private final TrcTriggerDigitalInput exitTrigger;
    private TrcEvent entryEvent;
    private TrcEvent exitEvent;
    private int numObjects = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     * @param entrySensor specifies the sensor object for the conveyor entry.
     * @param exitSensor specifies the sensor object for the conveyor exit.
     * @param params specifies the parameters object.
     */
    public TrcPidConveyor(
        String instanceName, TrcMotor motor, TrcDigitalInput entrySensor, TrcDigitalInput exitSensor,
        Parameters params)
    {
        this.motor = motor;
        this.entrySensor = entrySensor;
        this.exitSensor = exitSensor;
        this.params = params;

        if (entrySensor != null)
        {
            entryTrigger = new TrcTriggerDigitalInput(instanceName + ".entyTrigger", entrySensor);
            entryTrigger.enableTrigger(TriggerMode.OnBoth, this::onEntryEvent);
        }
        else
        {
            entryTrigger = null;
        }

        if (exitSensor != null)
        {
            exitTrigger = new TrcTriggerDigitalInput(instanceName + ".exitTrigger", exitSensor);
            exitTrigger.enableTrigger(TriggerMode.OnBoth, this::onExitEvent);
        }
        else
        {
            exitTrigger = null;
        }
    }   //TrcPidConveyor

    /**
     * This method returns the number of objects in the conveyor.
     * Note: Conveyor can only keep track of the number of objects if it has entry and exit sensors. Otherwise,
     * the number of objects returned will not be correct.
     *
     * @return number of objects in the conveyor.
     */
    public int getNumObjects()
    {
        return numObjects;
    }   //getNumObjects

    /**
     * This method sets the number of preloaded objects in the conveyor.
     *
     * @param num specifies the number of preloaded objects in the conveyor.
     */
    public void setPreloadedObjects(int num)
    {
        this.numObjects = num;
    }   //setPreloadedObjects

    /**
     * This method registers an event to be signaled when the entry sensor is triggered.
     *
     * @param event specifies event to signal when an object has entered the conveyor.
     */
    public void registerEntryEvent(TrcEvent event)
    {
        if (entryTrigger != null)
        {
            entryEvent = event;
        }
    }   //registerEntryEvent

    /**
     * This method registers an event to be signaled when the exit sensor is triggered.
     *
     * @param event specifies event to signal when an object has exited the conveyor.
     */
    public void registerExitEvent(TrcEvent event)
    {
        if (exitTrigger != null)
        {
            exitEvent = event;
        }
    }   //registerExitEvent

    /**
     * This method returns the sensor state read from the digital sensor.
     *
     * @return digital sensor state.
     */
    public boolean isEntrySensorActive()
    {
        return entrySensor != null && entrySensor.isActive();
    }   //isEntrySensorActive

    /**
     * This method returns the sensor state read from the digital sensor.
     *
     * @return digital sensor state.
     */
    public boolean isExitSensorActive()
    {
        return exitSensor != null && exitSensor.isActive();
    }   //isExitSensorActive

    /**
     * This methods sets the conveyor motor power. This is useful if the entry sensor is positioned a little way back
     * such that the convery needs to be ON and transport the 
     * @param owner
     * @param power
     */
    public void setPower(String owner, double power)
    {
        if (validateOwnership(owner))
        {
            if (numObjects == 0 || power <= 0.0)
            {
                motor.setPower(power);
            }
        }
    }   //setPower

    /**
     * This method advances or backs up the conveyor by the number of object units.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the conveyor.
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
            motor.setPosition(0.0, units*params.objectDistance, false, 1.0, event);
        }
    }   //move

    /**
     * This method advances or backs up the conveyor by the number of object units.
     *
     * @param units specifies the number of object units to advance or negative number to back up.
     * @param event specifies the event to signal when done, can be null if not provided.
     */
    public void move(int units, TrcEvent event)
    {
        move(null, units, event);
    }   //move

    /**
     * This method advances or backs up the conveyor by the number of object units.
     *
     * @param units specifies the number of object units to advance or negative number to back up.
     */
    public void move(int units)
    {
        move(null, units, null);
    }   //move

    /**
     * This method advances the conveyor by the one object unit.
     */
    public void advance()
    {
        move(null, 1, null);
    }   //advance

    /**
     * This method backs up the conveyor by the one object unit.
     */
    public void backup()
    {
        move(null, -1, null);
    }   //backup

    /**
     * This method is called when the entry sensor is triggered.
     *
     * @param context specifies true if an object has activated the sensor, false if the object has deactivated it.
     */
    private void onEntryEvent(Object context)
    {
        boolean active = ((AtomicBoolean) context).get();

        if (active)
        {
            numObjects++;
            motor.setPower(params.movePower);
        }
        else
        {
            motor.setPower(0.0);
        }

        if (entryEvent != null)
        {
            entryEvent.signal();
        }
    }   //onEntryEvent

    /**
     * This method is called when the exit sensor is triggered.
     *
     * @param context specifies true if an object has activated the sensor, false if the object has deactivated it.
     */
    private void onExitEvent(Object context)
    {
        boolean active = ((AtomicBoolean) context).get();

        if (active)
        {
            numObjects--;
        }
        else
        {
            motor.setPower(0.0);
        }

        if (exitEvent != null)
        {
            exitEvent.signal();
        }
    }   //onExitEvent

}   //class TrcPidConveyor
