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

import java.util.Arrays;

import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;
import trclib.robotcore.TrcPresets;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent Differential Servo Wrist Subsystem. A Differential Servo Wrist consists
 * of two servos controlling a 2-DOF wrist. The wrist can tilt as well as rotate. When the two servos turn in the same
 * direction on the mounted axis, the wrist tilts up and down. When the two servos turn in opposite directions, the
 * wrist rotates.
 */
public class TrcDifferentialServoWrist implements TrcExclusiveSubsystem
{
    /**
     * This class contains all the parameters of the Differential Servo Wrist.
     */
    public static class WristParams
    {
        private double logicalMin = 0.0;
        private double logicalMax = 1.0;
        private double physicalPosRange = 1.0;
        private double tiltPosOffset = 0.0;
        private double rotatePosOffset = 0.0;
        private double maxStepRate = 0.0;
        private double tiltPosLowLimit = 0.0;
        private double tiltPosHighLimit = 1.0;
        private double rotatePosLowLimit = 0.0;
        private double rotatePosHighLimit = 1.0;
        private double presetTolerance = 0.0;
        private double[] tiltPosPresets = null;
        private double[] rotatePosPresets = null;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @Override
        public String toString()
        {
            return "(logRangeMin=" + logicalMin +
                   ",logRangeMax=" + logicalMax +
                   ",phyRange=" + physicalPosRange +
                   ",tiltOffset=" + tiltPosOffset +
                   ",rotateOffset=" + rotatePosOffset +
                   ",maxStepRate=" + maxStepRate +
                   ",tiltPosLowLimit=" + tiltPosLowLimit +
                   ",tiltPosHighLimit=" + tiltPosHighLimit +
                   ",rotatePosLowLimit=" + rotatePosLowLimit +
                   ",rotatePosHighLimit=" + rotatePosHighLimit +
                   ",presetTolerance=" + presetTolerance +
                   ",tiltPosPresets=" + (tiltPosPresets != null? Arrays.toString(tiltPosPresets): "null") +
                   ",rotatePosPresets=" + (rotatePosPresets != null? Arrays.toString(rotatePosPresets): "null" + ")");
        }   //toString

        /**
         * This method sets the physical tilt and rotate position range of the wrist. Because of the nature of
         * differential wrist, tilt position range must be the same as rotate range.
         *
         * @param logicalMin specifies the logical minimum value of the servos. Typically 0.0.
         * @param logicalMax specifies the logical maximum value of the servos. Typically 1.0.
         * @param physicalRange specifies the physical position range, typically the servo movement range in degrees.
         *        For example, for 180-degree servo, physicalPosRange will be 180.0.
         * @param tiltOffset specifies the tilt position offset. This is the offset from physical zero position to
         *        the tilt range center position. For example, if the tilt range center is 45 degrees below physical
         *        zero position, tiltPosOffset will be -45.0.
         * @param rotateOffset specifies the rotate position offset. This is the offset from physical zero position
         *        to the rotate range center position. For example, if the rotate range center is exactly physical
         *        zero position, rotatePosOffset will be 0.0.
         * @return this object for chaining.
         */
        public WristParams setPosRange(
            double logicalMin, double logicalMax, double physicalRange, double tiltOffset, double rotateOffset)
        {
            this.logicalMin = logicalMin;
            this.logicalMax = logicalMax;
            this.physicalPosRange = physicalRange;
            this.tiltPosOffset = tiltOffset;
            this.rotatePosOffset = rotateOffset;
            return this;
        }   //setPosRange

        /**
         * This method sets the maximum speed of the servo.
         *
         * @param maxStepRate specifies the maximum speed of the servo in degrees per second.
         * @return this object for chaining.
         */
        public WristParams setMaxStepRate(double maxStepRate)
        {
            this.maxStepRate = maxStepRate;
            return this;
        }   //setMaxStepRate

        /**
         * This method sets the tilt and rotate position limits.
         *
         * @param tiltPosLowLimit specifies the tilt position low limit in physical unit.
         * @param tiltPosHighLimit specifies the tilt position high limit in physical unit.
         * @param rotatePosLowLimit specifies the rotate position low limit in physical unit.
         * @param rotatePosHighLimit specifies the rotate position high limit in physical unit.
         * @return this object for chaining.
         */
        public WristParams setPositionLimits(
            double tiltPosLowLimit, double tiltPosHighLimit, double rotatePosLowLimit, double rotatePosHighLimit)
        {
            this.tiltPosLowLimit = tiltPosLowLimit;
            this.tiltPosHighLimit = tiltPosHighLimit;
            this.rotatePosLowLimit = rotatePosLowLimit;
            this.rotatePosHighLimit = rotatePosHighLimit;
            return this;
        }   //setPositionLimits

        /**
         * This method sets the position preset parameters for both tilt and rotate.
         *
         * @param presetTolerance specifies the preset tolerance.
         * @param tiltPosPresets specifies the tilt position preset array.
         * @param rotatePosPresets specifies the rotate position preset array.
         * @return this object for chaining.
         */
        public WristParams setPosPresets(double presetTolerance, double[] tiltPosPresets, double[] rotatePosPresets)
        {
            this.presetTolerance = presetTolerance;
            this.tiltPosPresets = tiltPosPresets;
            this.rotatePosPresets = rotatePosPresets;
            return this;
        }   //setPosPresets

    }   //class WristParams

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
                   ",owner=" + owner +
                   ",tiltValue=" + tiltValue +
                   ",rotateValue=" + rotateValue +
                   ",event=" + event +
                   ",timeout=" + timeout + ")";
        }   //toString

    }   //class ActionParams

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcServo servo1, servo2;
    private final WristParams wristParams;
    private final TrcTimer timer;
    private final TrcPresets tiltPosPresets;
    private final TrcPresets rotatePosPresets;

    private ActionParams actionParams = null;
    private double tiltPower = 0.0;
    private double rotatePower = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param servo1 specifies the servo 1 object.
     * @param servo2 specifies the servo 2 object.
     * @param wristParams specifies the wrist parameters.
     */
    public TrcDifferentialServoWrist(String instanceName, TrcServo servo1, TrcServo servo2, WristParams wristParams)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.wristParams = wristParams;
        this.timer = new TrcTimer(instanceName);
        this.tiltPosPresets =
            wristParams.tiltPosPresets != null?
                new TrcPresets(
                    instanceName + ".tiltPosPresets", wristParams.presetTolerance, wristParams.tiltPosPresets):
                null;
        this.rotatePosPresets =
            wristParams.rotatePosPresets != null?
                new TrcPresets(
                    instanceName + ".rotatePosPresets", wristParams.presetTolerance, wristParams.rotatePosPresets):
                null;

        double halfPosRange = wristParams.physicalPosRange / 2.0;
        servo1.setLogicalPosRange(wristParams.logicalMin, wristParams.logicalMax);
        servo2.setLogicalPosRange(wristParams.logicalMin, wristParams.logicalMax);
        servo1.setPhysicalPosRange(-halfPosRange, halfPosRange);
        servo2.setPhysicalPosRange(-halfPosRange, halfPosRange);
        servo1.setMaxStepRate(wristParams.maxStepRate);
        servo2.setMaxStepRate(wristParams.maxStepRate);
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
     * @param canceled specifies true if canceled.
     */
    private void performAction(Object context, boolean canceled)
    {
        if (!canceled)
        {
            ActionParams actionParams = (ActionParams) context;
            double currServo1Pos = servo1.getPosition();
            double currServo2Pos = servo2.getPosition();
            double currTiltPos = getTiltPosition(currServo1Pos, currServo2Pos);
            double currRotatePos = getRotatePosition(currServo1Pos, currServo2Pos);
            // Because of the nature of Differential Wrist, it has two DOFs (tilt and rotate) that interact with
            // each other. When implemented using regular servos which have a movement range restriction (e.g.
            // 180-degree servos), the position of one DOF will impose movement restriction on the other DOF. For
            // example, if both tilt and rotate has a max movement range of -90 degrees to +90 degrees, when tilt is
            // at one extreme limit (e.g. +90 degree), the servos cannot move further past that limit. It means
            // rotate will be stuck at zero degree. If the tilt is at +45 degrees, it has a headroom of 45 degrees
            // before it will hit the 90-degree limit and therefore, rotate has a range of -45 to 45 degrees. If you
            // try to rotate beyond the 45-degree restriction, tilt will be forced to move. In other words, rotating
            // the wrist may make the wrist tilt position to change unexpectedly. This is not desirable. To solve
            // this dilemma, we decided to prioritize tilt position over rotate position. It means we impose no
            // restriction on tilt position but rotate position will be restricted to the headroom range imposed by
            // tilt position. If the wrist is at 0-degree, rotate will have full rotate range (-90 to +90). Tilting
            // the wrist in either direction will decrease the headroom and thus restricting rotation to a smaller
            // range.
            tracer.traceInfo(
                instanceName,
                "currTiltPos=" + currTiltPos + ",currRotatePos=" + currRotatePos + ",actionParams=" + actionParams);
            if (actionParams.operation == Operation.SetPower)
            {
                // If going outside the limits, stop it.
                tiltPower =
                    actionParams.tiltValue > 0.0 && currTiltPos > wristParams.tiltPosHighLimit ||
                    actionParams.tiltValue < 0.0 && currTiltPos < wristParams.tiltPosLowLimit?
                        0.0: actionParams.tiltValue;
                rotatePower =
                    actionParams.rotateValue > 0.0 && currRotatePos > wristParams.rotatePosHighLimit ||
                    actionParams.rotateValue < 0.0 && currRotatePos < wristParams.rotatePosLowLimit?
                        0.0: actionParams.rotateValue;
                double mag = TrcUtil.magnitude(tiltPower, rotatePower);
                if (mag > 1.0)
                {
                    tiltPower /= mag;
                    rotatePower /= mag;
                }

                double servo1Power = tiltPower + rotatePower;
                double servo2Power = tiltPower - rotatePower;
                double currTiltOffset = currTiltPos - wristParams.tiltPosOffset;
                double maxHeadroom = wristParams.physicalPosRange / 2.0;
                double headroom = maxHeadroom - Math.abs(currTiltOffset);
                if (tiltPower != 0.0)
                {
                    // Don't restrict the range if we are tilting the wrist.
                    servo1.setPower(servo1Power, -maxHeadroom, maxHeadroom);
                    servo2.setPower(servo2Power, -maxHeadroom, maxHeadroom);
                }
                else
                {
                    // Restrict the range if we are just rotating.
                    servo1.setPower(servo1Power, currTiltOffset - headroom, currTiltOffset + headroom);
                    servo2.setPower(servo2Power, currTiltOffset - headroom, currTiltOffset + headroom);
                }
                tracer.traceDebug(
                    instanceName,
                    "setPower(tiltPwr=%.3f,rotatePwr=%.3f,servo1Pwr=%.3f,servo2Pwr=%.3f,currTiltOffset=%.3f," +
                    "headroom=%.3f,currServo1Pos=%.3f,currServo2Pos=%.3f)",
                    tiltPower, rotatePower, servo1Power, servo2Power, currTiltOffset, headroom, currServo1Pos,
                    currServo2Pos);
                finish(true);
            }
            else if (actionParams.operation == Operation.SetPosition)
            {
                double targetTiltPos = TrcUtil.clipRange(
                    actionParams.tiltValue, wristParams.tiltPosLowLimit, wristParams.tiltPosHighLimit);
                double targetRotatePos = TrcUtil.clipRange(
                    actionParams.rotateValue, wristParams.rotatePosLowLimit, wristParams.rotatePosHighLimit);
                double headroom =
                    wristParams.physicalPosRange / 2.0 - Math.abs(targetTiltPos - wristParams.tiltPosOffset);
                // Restricting rotate position further according to tilt position.
                targetRotatePos = TrcUtil.clipRange(
                    targetRotatePos, -headroom + wristParams.rotatePosOffset, headroom + wristParams.rotatePosOffset);
                double servo1TargetPos = getServo1Position(targetTiltPos, targetRotatePos);
                double servo2TargetPos = getServo2Position(targetTiltPos, targetRotatePos);
                servo1.setPosition(servo1TargetPos);
                servo2.setPosition(servo2TargetPos);
                tracer.traceDebug(
                    instanceName,
                    "setPosition(tilt=%.3f,rotate=%.3f,headroom=%.3f,servo1Target=%.3f,servo2Target=%.3f)",
                    actionParams.tiltValue, actionParams.rotateValue, headroom, servo1TargetPos, servo2TargetPos);

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
        }
    }   //performAction

    /**
     * This method is called when the action has timed out. Servo doesn't have feedback sensors. Timeout is used as
     * the way for the caller to wait for completion. Therefore, a timeout here means the operation has completed.
     *
     * @param context not used.
     * @param canceled not used.
     */
    private void actionTimedOut(Object context, boolean canceled)
    {
        tracer.traceDebug(instanceName, "actionParams=%s", actionParams);
        finish(true);
    }   //actionTimedOut

    /**
     * This method sets the wrist tilting/rotating power.
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
                performAction(actionParams, false);
            }
        }
    }   //setPower

    /**
     * This method sets the wrist tilting/rotating power.
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
     * This method sets the wrist tilting/rotating power.
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
                performAction(actionParams, false);
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
        return getTiltPosition(servo1.getPosition(), servo2.getPosition());
    }   //getTiltPosition

    /**
     * This method returns the physical rotate position value of the wrist. Generally, servo do not provide real time
     * position feedback. Therefore, it will only return the position set by the last setPosition call.
     *
     * @return physical rotate position of the wrist, could be in degrees if setPhysicalPosRange is called to set the
     *         range in degrees.
     */
    public double getRotatePosition()
    {
        return getRotatePosition(servo1.getPosition(), servo2.getPosition());
    }   //getRotatePosition

    private double getTiltPosition(double servo1Pos, double servo2Pos)
    {
        return (servo1Pos + servo2Pos) / 2.0 + wristParams.tiltPosOffset;
    }   //getTiltPosition

    private double getRotatePosition(double servo1Pos, double servo2Pos)
    {
        return (servo1Pos - servo2Pos) / 2.0 + wristParams.rotatePosOffset;
    }   //getRotatePosition

    private double getServo1Position(double tiltPos, double rotatePos)
    {
        return ((tiltPos - wristParams.tiltPosOffset) + (rotatePos - wristParams.rotatePosOffset));
    }   //getServo1Position

    private double getServo2Position(double tiltPos, double rotatePos)
    {
        return ((tiltPos - wristParams.tiltPosOffset) - (rotatePos - wristParams.rotatePosOffset));
    }   //getServo2Position

    //
    // Presets.
    //

    /**
     * This method sets the wrist to the specified tilt preset position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param presetIndex specifies the index to the preset position array.
     * @param event specifies the event to signal when target is reached, can be null if not provided.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setTiltPresetPosition(String owner, double delay, int presetIndex, TrcEvent event, double timeout)
    {
        if (tiltPosPresets != null && tiltPosPresets.validatePresetIndex(presetIndex))
        {
            setPosition(owner, delay, tiltPosPresets.getPresetValue(presetIndex), getRotatePosition(), event, timeout);
        }
    }   //setTiltPresetPosition

    /**
     * This method sets the wrist to the specified rotate preset position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param presetIndex specifies the index to the preset position array.
     * @param event specifies the event to signal when target is reached, can be null if not provided.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setRotatePresetPosition(String owner, double delay, int presetIndex, TrcEvent event, double timeout)
    {
        if (rotatePosPresets != null && rotatePosPresets.validatePresetIndex(presetIndex))
        {
            setPosition(owner, delay, getTiltPosition(), rotatePosPresets.getPresetValue(presetIndex), event, timeout);
        }
    }   //setRotatePresetPosition

    /**
     * This method sets the wrist to the next tilt preset position up or down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     * @param presetUp specifies true to move to next preset up, false to move to next preset down.
     */
    private void setNextTiltPresetPosition(String owner, boolean presetUp)
    {
        if (tiltPosPresets != null)
        {
            double currValue = getTiltPosition();
            int index = presetUp?
                tiltPosPresets.nextPresetIndexUp(currValue): tiltPosPresets.nextPresetIndexDown(currValue);

            if (index != -1)
            {
                setTiltPresetPosition(owner, 0.0, index, null, 0.0);
            }
        }
    }   //setNextTiltPresetPosition

    /**
     * This method sets the wrist to the next tilt preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void tiltPresetPositionUp(String owner)
    {
        setNextTiltPresetPosition(owner, true);
    }   //tiltPresetPositionUp

    /**
     * This method sets the wrist to the next tilt preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void tiltPresetPositionDown(String owner)
    {
        setNextTiltPresetPosition(owner, false);
    }   //tiltPresetPositionDown

    /**
     * This method sets the wrist to the next rotate preset position up or down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     * @param presetUp specifies true to move to next preset up, false to move to next preset down.
     */
    private void setNextRotatePresetPosition(String owner, boolean presetUp)
    {
        if (rotatePosPresets != null)
        {
            double currValue = getRotatePosition();
            int index = presetUp ?
                rotatePosPresets.nextPresetIndexUp(currValue) : rotatePosPresets.nextPresetIndexDown(currValue);

            if (index != -1)
            {
                setRotatePresetPosition(owner, 0.0, index, null, 0.0);
            }
        }
    }   //setNextRotatePresetPosition

    /**
     * This method sets the wrist to the next rotate preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void rotatePresetPositionUp(String owner)
    {
        setNextRotatePresetPosition(owner, true);
    }   //rotatePresetPositionUp

    /**
     * This method sets the wrist to the next rotate preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void rotatePresetPositionDown(String owner)
    {
        setNextRotatePresetPosition(owner, false);
    }   //rotatePresetPositionDown

}   //class TrcDifferentialServoWrist
