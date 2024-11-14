/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent Differential Servo Wrist Subsystem. A Differential Servo Wrist consists
 * of two servos controlling two degrees of freedom. The wrist can tilt as well as rotate. When the two servos turn
 * in the same direction on the mounted axis, the wrist tilts up and down. When the two servos turn in opposite
 * directions, the wrist rotates.
 */
public class TrcDifferentialServoWrist implements TrcExclusiveSubsystem
{
    /**
     * Specifies the operation types.
     */
    private enum Operation
    {
        SetPower,
        SetPosition
    }   //enum Operation

    /**
     * This class encapsulates all the parameters required to perform the intake action.
     */
    private static class ActionParams
    {
        Operation operation;
        String owner;
        double tiltValue;
        double rotateValue;
        TrcEvent event;
        double timeout;

        ActionParams(
            Operation operation, String owner, double tiltValue, double rotateValue, TrcEvent event, double timeout)
        {
            this.operation = operation;
            this.owner = owner;
            this.tiltValue = tiltValue;
            this.rotateValue = rotateValue;
            this.event = event;
            this.timeout = timeout;
        }   //ActionParams

        @Override
        public String toString()
        {
            return "(op=" + operation +
                   ", owner=" + owner +
                   ", tiltValue=" + tiltValue +
                   ", rotateValue=" + rotateValue +
                   ", event=" + event +
                   ", timeout=" + timeout + ")";
        }   //toString

    }   //class ActionParams

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcServo servo1, servo2;
    private final TrcTimer timer;
    private ActionParams actionParams = null;
    private double tiltPower = 0.0;
    private double rotatePower = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param servo1 specifies the servo 1 object.
     * @param servo2 specifies the servo 2 object.
     */
    public TrcDifferentialServoWrist(String instanceName, TrcServo servo1, TrcServo servo2)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.timer = new TrcTimer(instanceName);
    }   //TrcDifferentialServoWrist

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
     * This method is called to finish the operation and to clean up. It can be called either at the end of timeout
     * or when the operation is done and signal the caller for completion. It can also be called if the caller
     * explicitly cancel the operation in which case the event will be set to canceled.
     *
     * @param completed specifies true if the operation is completed, false if canceled.
     */
    private void finish(boolean completed)
    {
        tracer.traceInfo(instanceName, "completed=" + completed + ", actionParams=" + actionParams);
        if (actionParams != null)
        {
            if (!completed)
            {
                timer.cancel();
                servo1.cancel(actionParams.owner);
                servo2.cancel(actionParams.owner);
            }

            if (actionParams.event != null)
            {
                if (completed)
                {
                    actionParams.event.signal();
                }
                else
                {
                    actionParams.event.cancel();
                }
                actionParams.event = null;
            }
            actionParams = null;
        }
    }   //finish

    /**
     * This method cancels previous wrist operation if applicable.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     */
    public void cancel(String owner)
    {
        tracer.traceInfo(instanceName, "owner=" + owner);
        if (validateOwnership(owner))
        {
            finish(false);
        }
    }   //cancel

    /**
     * This method cancels previous wrist operation if applicable.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method performs the action.
     *
     * @param context specifies the action parameters.
     */
    private void performAction(Object context)
    {
        ActionParams actionParams = (ActionParams) context;

        tracer.traceInfo(instanceName, "actionParams=" + actionParams);
        if (actionParams.operation == Operation.SetPower)
        {
            tiltPower = actionParams.tiltValue;
            rotatePower = actionParams.rotateValue;
            double mag = TrcUtil.magnitude(tiltPower, rotatePower);
            if (mag > 1.0)
            {
                tiltPower /= mag;
                rotatePower /= mag;
            }
            servo1.setPower(tiltPower + rotatePower);
            servo2.setPower(tiltPower - rotatePower);
            finish(true);
        }
        else if (actionParams.operation == Operation.SetPosition)
        {
            double tiltPos = actionParams.tiltValue;
            double rotatePos = actionParams.rotateValue;
            double mag = TrcUtil.magnitude(tiltPos, rotatePos);
            if (mag > 1.0)
            {
                tiltPos /= mag;
                rotatePos /= mag;
            }
            servo1.setPosition(tiltPos + rotatePos);
            servo2.setPosition(tiltPos - rotatePos);
            if (actionParams.timeout > 0.0)
            {
                timer.set(actionParams.timeout, this::actionTimedOut);
            }
            else
            {
                // No timeout provided, signal completion immediately.
                finish(true);
            }
        }
    }   //performAction

    /**
     * This method is called when the action has timed out. Servo doesn't have feedback sensors. Timeout is used as
     * the way for the caller to wait for completion. Therefore, a timeout here means the operation has completed.
     *
     * @param context not used.
     */
    private void actionTimedOut(Object context)
    {
        tracer.traceDebug(instanceName, "actionParams=" + actionParams);
        finish(true);
    }   //actionTimedOut

    /**
     * This method sets the wrist tilting power.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the power of the wrist, can be zero if no delay.
     * @param tiltPower specifies how fast the wrist will tilt.
     * @param rotatePower specifies how fast the wrist will rotate.
     */
    public void setPower(String owner, double delay, double tiltPower, double rotatePower)
    {
        tracer.traceDebug(
            instanceName, "setPower(owner=%s, delay=%.3f, tiltPower=%.1f, rotatePower=%.1f)",
            owner, delay, tiltPower, rotatePower);

        if (validateOwnership(owner))
        {
            actionParams = new ActionParams(Operation.SetPower, owner, tiltPower, rotatePower, null, 0.0);
            if (delay > 0.0)
            {
                timer.set(delay, this::performAction, actionParams);
            }
            else
            {
                performAction(actionParams);
            }
        }
    }   //setPower

    /**
     * This method sets the wrist tilting power.
     *
     * @param delay specifies the delay in seconds before setting the power of the wrist, can be zero if no delay.
     * @param tiltPower specifies how fast the wrist will tilt.
     * @param rotatePower specifies how fast the wrist will rotate.
     */
    public void setPower(double delay, double tiltPower, double rotatePower)
    {
        setPower(null, delay, tiltPower, rotatePower);
    }   //setPower

    /**
     * This method sets the wrist tilting power.
     *
     * @param tiltPower specifies how fast the wrist will tilt.
     * @param rotatePower specifies how fast the wrist will rotate.
     */
    public void setPower(double tiltPower, double rotatePower)
    {
        setPower(null, 0.0, tiltPower, rotatePower);
    }   //setPower

    /**
     * This method returns the last set tilt power value.
     *
     * @return last tilt power set to the wrist.
     */
    public double getTiltPower()
    {
        return tiltPower;
    }   //getTiltPower

    /**
     * This method returns the last set rotate power value.
     *
     * @return last rotate power set to the wrist.
     */
    public double getRotatePower()
    {
        return rotatePower;
    }   //getRotatePower

    /**
     * This method sets the tilt position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the tilt position of the wrist, can be zero if no
     *        delay.
     * @param tiltPos specifies the physical tilt position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param rotatePos specifies the physical rotate position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(
        String owner, double delay, double tiltPos, double rotatePos, TrcEvent completionEvent, double timeout)
    {
        tracer.traceDebug(
            instanceName, "setPosition(owner=%s, delay=%.3f, tiltPos=%.3f, rotatePos=%.3f, event=%s, timeout=%.3f)",
            owner, delay, tiltPos, rotatePos, completionEvent, timeout);

        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;
        //
        // Make sure the caller has ownership.
        //
        if (validateOwnership(owner))
        {
            actionParams = new ActionParams(Operation.SetPosition, owner, tiltPos, rotatePos, completionEvent, timeout);
            if (delay > 0.0)
            {
                timer.set(delay, this::performAction, actionParams);
            }
            else
            {
                performAction(actionParams);
            }
        }
    }   //setPosition

    /**
     * This method sets the tilt position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param delay specifies the delay in seconds before setting the tilt position of the wrist, can be zero if no
     *        delay.
     * @param tiltPos specifies the physical tilt position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param rotatePos specifies the physical rotate position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(double delay, double tiltPos, double rotatePos, TrcEvent completionEvent, double timeout)
    {
        setPosition(null, delay, tiltPos, rotatePos, completionEvent, timeout);
    }   //setPosition

    /**
     * This method sets the tilt position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param tiltPos specifies the physical tilt position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param rotatePos specifies the physical rotate position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(double tiltPos, double rotatePos, TrcEvent completionEvent, double timeout)
    {
        setPosition(null, 0.0, tiltPos, rotatePos, completionEvent, timeout);
    }   //setPosition

    /**
     * This method sets the tilt position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param tiltPos specifies the physical tilt position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param rotatePos specifies the physical rotate position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     */
    public void setPosition(double tiltPos, double rotatePos)
    {
        setPosition(null, 0.0, tiltPos, rotatePos, null, 0.0);
    }   //setPosition

    /**
     * This method returns the physical tilt position value of the wrist. Generally, servo do not provide real time
     * position feedback. Therefore, it will only return the position set by the last setPosition call.
     *
     * @return physical tilt position of the wrist, could be in degrees if setPhysicalPosRange is called to set the
     *         range in degrees.
     */
    public double getTiltPosition()
    {
        return (servo1.getPosition() + servo2.getPosition()) / 2.0;
    }   //getRotatePosition

    /**
     * This method returns the physical rotate position value of the wrist. Generally, servo do not provide real time
     * position feedback. Therefore, it will only return the position set by the last setPosition call.
     *
     * @return physical rotate position of the wrist, could be in degrees if setPhysicalPosRange is called to set the
     *         range in degrees.
     */
    public double getRotatePosition()
    {
        return (servo1.getPosition() - servo2.getPosition()) / 2.0;
    }   //getRotatePosition

}   //class TrcDifferentialServoWrist
