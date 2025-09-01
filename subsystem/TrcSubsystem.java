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

import java.util.ArrayList;

import trclib.robotcore.TrcEvent;

/**
 * This class implements a generic platform independent subsystem. It is intended to be extended by a subsystem class
 * that must implement a set of common subsystem abstract methods.
 */
public abstract class TrcSubsystem
{
    private static final ArrayList<SubsystemInfo> subsystemList = new ArrayList<>();
    private static TrcEvent zeroCalCompletionEvent = null;
    protected final String instanceName;

    private static class SubsystemInfo
    {
        final TrcSubsystem subsystem;
        Boolean zeroCalDone;

        SubsystemInfo(TrcSubsystem subsystem, Boolean zeroCalDone)
        {
            this.subsystem = subsystem;
            this.zeroCalDone = zeroCalDone;
        }   //SubsystemInfo

    }   //class SubsystemInfo

    //
    // Abstract methods to be implemented by subsystem classes.
    //

    /**
     * This method cancels any pending operations.
     */
    public abstract void cancel();

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    public abstract void zeroCalibrate(String owner, TrcEvent event);

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    public abstract void resetState();

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @return updated line number for the next subsystem to print.
     */
    public abstract int updateStatus(int lineNum);

    /**
     * This method is called to prep the subsystem for tuning.
     *
     * @params tuneParams specifies tuning parameters in an array of doubles.
     */
    public abstract void prepSubsystemForTuning(double... tuneParams);

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param needsZeroCal specifies true if the subsystem needs zero calibration, false otherwise.
     */
    public TrcSubsystem(String instanceName, boolean needsZeroCal)
    {
        this.instanceName = instanceName;
        subsystemList.add(new SubsystemInfo(this, needsZeroCal? false: null));
    }   //TrcSubsystem

    /**
     * This method returns the subsystem name.
     *
     * @return subsystem name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the subsystem matches the given name.
     *
     * @param name specifies the subsystem name to look for.
     * @return subsystem matching the given name.
     */
    public static TrcSubsystem getSubsystem(String name)
    {
        for (SubsystemInfo subsystemInfo: subsystemList)
        {
            if (subsystemInfo.subsystem.instanceName.equalsIgnoreCase(name))
            {
                return subsystemInfo.subsystem;
            }
        }

        return null;
    }   //getSubsystem

    /**
     * This method enumerates all subsystems and calls their cancel method.
     */
    public static void cancelAll()
    {
        for (SubsystemInfo subsystemInfo: subsystemList)
        {
            subsystemInfo.subsystem.cancel();
        }
    }   //cancelAll

    /**
     * This method enumerates all subsystems and calls their zeroCalibrate method.
     */
    public static void zeroCalibrateAll(String owner, TrcEvent completionEvent)
    {
        zeroCalCompletionEvent = completionEvent;
        for (SubsystemInfo subsystemInfo: subsystemList)
        {
            // Only do zero calibration for subsystems that need it.
            if (subsystemInfo.zeroCalDone != null)
            {
                TrcEvent event = completionEvent != null?
                    new TrcEvent(subsystemInfo.subsystem.instanceName + ".zeroCal"): null;
                if (event != null)
                {
                    event.setCallback(TrcSubsystem::zeroCalCallback, subsystemInfo);
                }
                subsystemInfo.zeroCalDone = false;
                subsystemInfo.subsystem.zeroCalibrate(owner, event);
            }
        }
    }   //zeroCalibrate

    /**
     * This method enumerates all subsystems and calls their resetState method.
     */
    public static void resetStateAll()
    {
        for (SubsystemInfo subsystemInfo: subsystemList)
        {
            subsystemInfo.subsystem.resetState();
        }
    }   //resetStateAll

    /**
     * This method enumerates all subsystems and calls their updateStatus method.
     */
    public static int updateStatusAll(int lineNum)
    {
        for (SubsystemInfo subsystemInfo: subsystemList)
        {
            lineNum = subsystemInfo.subsystem.updateStatus(lineNum);
        }

        return lineNum;
    }   //updateStatusAll

    /**
     * This method is called when the zero calibration completion event of a subsystem is signaled. If there are
     * multiple subsystems, it will check
     *
     * @param context specifies the SubsystemInfo object.
     * @param canceled not used.
     */
    private static void zeroCalCallback(Object context, boolean canceled)
    {
        boolean zeroCalCompleted = true;

        ((SubsystemInfo) context).zeroCalDone = true;
        // Check all subsystems that need zero calibration have completed calibration.
        for (SubsystemInfo subsystemInfo: subsystemList)
        {
            if (subsystemInfo.zeroCalDone != null && !subsystemInfo.zeroCalDone)
            {
                // Subsystem needs zero calibration but is not done yet.
                zeroCalCompleted = false;
                break;
            }
        }

        if (zeroCalCompleted)
        {
            if (canceled)
            {
                zeroCalCompletionEvent.cancel();
            }
            else
            {
                zeroCalCompletionEvent.signal();
            }
            zeroCalCompletionEvent = null;
        }
    }   //zeroCalCallback

}   //class TrcSubsystem