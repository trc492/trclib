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

package trclib.controller;

import java.util.EmptyStackException;
import java.util.Stack;

import trclib.dataprocessor.TrcWarpSpace;
import trclib.output.TrcDashboard;
import trclib.robotcore.TrcDbgTrace;
import trclib.dataprocessor.TrcUtil;
import trclib.robotcore.TrcDbgTrace.MsgLevel;
import trclib.sensor.TrcRobotBattery;
import trclib.timer.TrcTimer;

/**
 * This class implements a PID controller. A PID controller takes a target set point and an input from a feedback
 * device to calculate the output power of an effector usually a motor or a set of motors. It also supports Feed
 * Forward if provided. Feed Forward control does not rely on any feedback devices, it derives its output from
 * velocity and/or acceleration set points.
 */
public class TrcPidController
{
    public static final double DEF_SETTLING_TIME = 0.2;
    private static final double DEF_STALL_DETECTION_DELAY = 0.5;
    private static final double DEF_STALL_DETECTION_TIMEOUT = 0.2;
    private static final double DEF_STALL_ERR_RATE_THRESHOLD = 1.0;

    /**
     * This class encapsulates all the Pid coefficients into a single object and makes it more efficient to pass
     * them around. All coefficients are declared final, so they cannot change once created. If you need to change
     * any coefficient values, you must create a new instance.
     */
    public static class PidCoefficients
    {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final double kS;
        public final double kV;
        public final double kA;
        public final double iZone;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional coefficient.
         * @param kI specifies the Integral coefficient.
         * @param kD specifies the Differential coefficient.
         * @param kF specifies the Feed forward coefficient.
         * @param kS specifies the Static gain.
         * @param kV specifies the Velocity gain.
         * @param kA specifies the Acceleration gain.
         * @param iZone specifies the maximum magnitude of error to allow integral control.
         */
        public PidCoefficients(
            double kP, double kI, double kD, double kF, double kS, double kV, double kA, double iZone)
        {
            this.kP = Math.abs(kP);
            this.kI = Math.abs(kI);
            this.kD = Math.abs(kD);
            this.kF = Math.abs(kF);
            this.kS = kS;
            this.kV = Math.abs(kV);
            this.kA = Math.abs(kA);
            this.iZone = Math.abs(iZone);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional coefficient.
         * @param kI specifies the Integral coefficient.
         * @param kD specifies the Differential coefficient.
         * @param kF specifies the Feed forward coefficient.
         */
        public PidCoefficients(double kP, double kI, double kD, double kF)
        {
            this(kP, kI, kD, kF, 0.0, 0.0, 0.0, 0.0);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional coefficient.
         * @param kI specifies the Integral coefficient.
         * @param kD specifies the Differential coefficient.
         */
        public PidCoefficients(double kP, double kI, double kD)
        {
            this(kP, kI, kD, 0.0, 0.0, 0.0, 0.0, 0.0);
        }   //PidCoefficients

        /**
         * This method returns all PID coefficients in string form.
         *
         * @return PID coefficients string.
         */
        @Override
        public String toString()
        {
            return "(PIDFSVAZ:" + kP + "," + kI + "," + kD + "," + kF +
                   "," + kS + "," + kV + "," + kA + "," + iZone + ")";
        }   //toString

        /**
         * This method returns a copy of this object.
         *
         * @return a copy of this object.
         */
        @Override
        public PidCoefficients clone()
        {
            return new PidCoefficients(kP, kI, kD, kF, kS, kV, kA, iZone);
        }   //clone

    }   //class PidCoefficients

    /**
     * PID controller needs input from a feedback device for calculating the output power. Whoever is providing this
     * input must implement this interface.
     */
    public interface PidInput
    {
        /**
         * This method is called by the PID controller to get input data from the feedback device. The feedback
         * device can be motor encoders, gyro, ultrasonic sensor, light sensor etc.
         *
         * @return input value of the feedback device.
         */
        double get();

    }   //interface PidInput

    /**
     * This class stores the PID controller state.
     */
    private static class PidCtrlState
    {
        PidCoefficients pidCoeffs = null;

        double input = 0.0;
        Double posSetPoint = null;
        Double velSetPoint = null;
        Double accelSetPoint = null;

        Double timestamp = null;
        double deltaTime = 0.0;
        double posError = 0.0;
        double velError = 0.0;
        double totalPosError = 0.0;

        double pTerm = 0.0;
        double iTerm = 0.0;
        double dTerm = 0.0;
        double fTerm = 0.0;
        double sTerm = 0.0;
        double vTerm = 0.0;
        double aTerm = 0.0;
        double output = 0.0;

        Double inputModulus = null;
        double settlingStartTime = 0.0;
        double setPointSign = 1.0;
        double stallDetectionDelay = 0.0;
        double stallDetectionTimeout = 0.0;
        double stallErrorRateThreshold = 0.0;
        Double stallDetectionStartTime = null;
        // Tracing Config.
        boolean verbosePidInfo = false;
        TrcRobotBattery battery = null;

        /**
         * This method resets the PID controller state.
         */
        void reset()
        {
            settlingStartTime = 0.0;
            posSetPoint = null;
            setPointSign = 1.0;
            timestamp = null;
            deltaTime = 0.0;
            input = 0.0;
            posError = 0.0;
            velError = 0.0;
            totalPosError = 0.0;
            pTerm = 0.0;
            iTerm = 0.0;
            dTerm = 0.0;
            fTerm = 0.0;
            sTerm = 0.0;
            vTerm = 0.0;
            aTerm = 0.0;
            output = 0.0;
            stallDetectionStartTime = null;
        }   //reset

    }   //class PidCtrlState

    private final TrcDashboard dashboard = TrcDashboard.getInstance();
    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final PidInput pidInput;

    private boolean inverted = false;
    private boolean absSetPoint = false;
    private boolean noOscillation = false;
    private double minOutput = -1.0;
    private double maxOutput = 1.0;
    private double minIntegral = -1.0;
    private double maxIntegral = 1.0;
    private double outputLimit = 1.0;
    private Double rampRate = null;
    private final Stack<Double> outputLimitStack = new Stack<>();
    private final PidCtrlState pidCtrlState = new PidCtrlState();

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pidCoeffs specifies the PID coefficients.
     * @param pidInput specifies the method to call to get PID sensor input.
     */
    public TrcPidController(String instanceName, PidCoefficients pidCoeffs, PidInput pidInput)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.pidInput = pidInput;
        synchronized (pidCtrlState)
        {
            pidCtrlState.pidCoeffs = pidCoeffs;
        }
    }   //TrcPidController

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
     * This method sets the message trace level for the tracer.
     *
     * @param msgLevel specifies the message level.
     * @param verbosePidInfo specifies true to trace verbose PID info, false otherwise.
     * @param battery specifies the battery object to get battery info for the message.
     */
    public void setTraceLevel(TrcDbgTrace.MsgLevel msgLevel, boolean verbosePidInfo, TrcRobotBattery battery)
    {
        synchronized (pidCtrlState)
        {
            tracer.setTraceLevel(msgLevel);
            pidCtrlState.verbosePidInfo = verbosePidInfo;
            pidCtrlState.battery = battery;
        }
    }   //setTraceLevel

    /**
     * This method sets the message trace level for the tracer.
     *
     * @param msgLevel specifies the message level.
     * @param verbosePidInfo specifies true to trace verbose PID info, false otherwise.
     */
    public void setTraceLevel(TrcDbgTrace.MsgLevel msgLevel, boolean verbosePidInfo)
    {
        setTraceLevel(msgLevel, verbosePidInfo, null);
    }   //setTraceLevel

    /**
     * This method sets the message trace level for the tracer.
     *
     * @param msgLevel specifies the message level.
     */
    public void setTraceLevel(TrcDbgTrace.MsgLevel msgLevel)
    {
        setTraceLevel(msgLevel, false, null);
    }   //setTraceLevel

    /**
     * This method returns the PidInput interface.
     *
     * @return PidInput interface.
     */
    public PidInput getPidInput()
    {
        return pidInput;
    }   //getPidInput

    /**
     * This method inverts the sign of the calculated error. Normally, the calculated error starts with a large
     * positive number and goes down. However, in some sensors such as the ultrasonic sensor, the target is a small
     * number and the error starts with a negative value and increases. In order to calculate a correct output which
     * will go towards the target, the error sign must be inverted.
     *
     * @param inverted specifies true to invert the sign of the calculated error, false otherwise.
     */
    public void setInverted(boolean inverted)
    {
        synchronized (pidCtrlState)
        {
            this.inverted = inverted;
        }
    }   //setInverted

    /**
     * This method sets the set point mode to be absolute. PID controller always calculates the output with an
     * absolute set point comparing to a sensor value representing an absolute input. But by default, it will
     * treat the set point as a value relative to its current input. So it will add the relative set point value
     * to the current input as the absolute set point in its calculation. This method allows the caller to treat
     * the set point as absolute set point.
     *
     * @param absolute specifies true if set point is absolute, false otherwise.
     */
    public void setAbsoluteSetPoint(boolean absolute)
    {
        synchronized (pidCtrlState)
        {
            this.absSetPoint = absolute;
        }
    }   //setAbsoluteSetPoint

    /**
     * This method returns true if setpoints are absolute, false otherwise.
     *
     * @return true if setpoints are absolute, false otherwise.
     */
    public boolean hasAbsoluteSetPoint()
    {
        return absSetPoint;
    }   //hasAbsoluteSetPoint

    /**
     * This method enables/disables NoOscillation mode. In PID control, if the PID constants are not tuned quite
     * correctly, it may cause oscillation that could waste a lot of time. In some scenarios, passing the target
     * beyond the tolerance may be acceptable. This method allows the PID controller to declare "On Target" even
     * though it passes the target beyond tolerance so it doesn't oscillate.
     *
     * @param noOscillation specifies true to enable no oscillation, false to disable.
     */
    public void setNoOscillation(boolean noOscillation)
    {
        synchronized (pidCtrlState)
        {
            this.noOscillation = noOscillation;
        }
    }   //setNoOscillation

    /**
     * This method enables/disables stall detection.
     *
     * @param stallDetectionDelay specifies stall detection start delay in seconds, zero to disable stall detection.
     * @param stallDetectionTimeout specifies stall timeout in seconds which is the minimum elapsed time for the
     *        motor to be motionless to be considered stalled.
     * @param stallErrorRateThreshold specifies the error rate threshold below which it will consider stalling.
     */
    public void setStallDetectionEnabled(
        double stallDetectionDelay, double stallDetectionTimeout, double stallErrorRateThreshold)
    {
        synchronized (pidCtrlState)
        {
            pidCtrlState.stallDetectionDelay = Math.abs(stallDetectionDelay);
            pidCtrlState.stallDetectionTimeout = Math.abs(stallDetectionTimeout);
            pidCtrlState.stallErrorRateThreshold = Math.abs(stallErrorRateThreshold);
        }
    }   //setStallDetectionEnabled

    /**
     * This method enables/disables stall detection.
     *
     * @param enabled specifies true to enable stall detection, false to disable.
     */
    public void setStallDetectionEnabled(boolean enabled)
    {
        if (enabled)
        {
            setStallDetectionEnabled(
                DEF_STALL_DETECTION_DELAY, DEF_STALL_DETECTION_TIMEOUT, DEF_STALL_ERR_RATE_THRESHOLD);
        }
        else
        {
            setStallDetectionEnabled(0.0, 0.0, 0.0);
        }
    }   //setStallDetectionEnabled

    /**
     * This method starts stall detection.
     */
    public void startStallDetection()
    {
        synchronized (pidCtrlState)
        {
            // Start stall detection only if it's not already started.
            if (pidCtrlState.stallDetectionStartTime == null)
            {
                pidCtrlState.stallDetectionStartTime =
                    pidCtrlState.stallDetectionTimeout == 0.0 ?
                        null : TrcTimer.getCurrentTime() + pidCtrlState.stallDetectionDelay;
            }
        }
    }   //startStallDetection

    /**
     * This method ends stall detection.
     */
    public void endStallDetection()
    {
        synchronized (pidCtrlState)
        {
            pidCtrlState.stallDetectionStartTime = null;
        }
    }   //endStallDetection

    /**
     * This method returns the current PID coefficients.
     *
     * @return current PID coefficients.
     */
    public PidCoefficients getPidCoefficients()
    {
        synchronized (pidCtrlState)
        {
            return pidCtrlState.pidCoeffs;
        }
    }   //getPidCoefficients

    /**
     * This method sets new PID coefficients.
     *
     * @param pidCoeffs specifies new PID coefficients.
     */
    public void setPidCoefficients(PidCoefficients pidCoeffs)
    {
        synchronized (pidCtrlState)
        {
            pidCtrlState.pidCoeffs = pidCoeffs;
        }
    }   //setPidCoefficients

    /**
     * This method sets the ramp rate of the PID controller output. It is sometimes useful to limit the acceleration
     * of the output of the PID controller. For example, the strafing PID controller on a mecanum drive base may
     * benefit from a lower acceleration to minimize wheel slippage.
     *
     * @param rampRate specifies the ramp rate in percent power per second, null to disable.
     */
    public void setRampRate(Double rampRate)
    {
        synchronized (pidCtrlState)
        {
            this.rampRate = rampRate;
        }
    }   //setRampRate

    /**
     * This method sets a range limit on the calculated output. It is very useful to limit the output range to
     * less than full power for scenarios such as using mecanum wheels on a drive train to prevent wheel slipping
     * or slow down a PID drive in order to detect a line etc.
     *
     * @param minOutput specifies the PID output lower range limit.
     * @param maxOutput specifies the PID output higher range limit.
     */
    public void setOutputRange(double minOutput, double maxOutput)
    {
        if (maxOutput <= minOutput)
        {
            throw new IllegalArgumentException("maxOutput must be greater than minOutput");
        }

        synchronized (pidCtrlState)
        {
            if (Math.abs(minOutput) == Math.abs(maxOutput))
            {
                outputLimit = maxOutput;
            }

            this.minOutput = minOutput;
            this.maxOutput = maxOutput;
        }
    }   //setOutputRange

    /**
     * This method sets a range limit on the integral output.
     *
     * @param minIntegral specifies the integral output lower limit.
     * @param maxIntegral specifies the integral output higher limit.
     */
    public void setIntegralRange(double minIntegral, double maxIntegral)
    {
        if (maxIntegral <= minIntegral)
        {
            throw new IllegalArgumentException("maxIntegral must be greater than minIntegral.");
        }

        synchronized (pidCtrlState)
        {
            this.minIntegral = minIntegral;
            this.maxIntegral = maxIntegral;
        }
    }   //setIntegralRange

    /**
     * This method sets the output to the range -limit to +limit. It calls setOutputRange. If the caller wants
     * to limit the output power symmetrically, this is the method to call, not setOutputRange.
     *
     * @param limit specifies the output limit as a positive number.
     */
    public void setOutputLimit(double limit)
    {
        synchronized (pidCtrlState)
        {
            limit = Math.abs(limit);
            setOutputRange(-limit, limit);
        }
    }   //setOutputLimit

    /**
     * This method returns the last set output limit. It is sometimes useful to temporarily change the output
     * range of the PID controller for an operation and restore it afterwards. This method allows the caller to
     * save the last set output limit and restore it later on.
     *
     * @return last set output limit.
     */
    public double getOutputLimit()
    {
        return outputLimit;
    }   //getOutputLimit

    /**
     * This method saves the current output limit of the PID controller and sets it to the given new limit.
     * This is useful if the caller wants to temporarily set a limit for an operation and restore it afterwards.
     * Note: this is implemented with a stack so it is assuming the saving and restoring calls are nested in
     * nature. If this is called in a multi-threading environment where saving and restoring can be interleaved
     * by different threads, unexpected result may happen. It is recommended to avoid this type of scenario if
     * possible.
     *
     * @param limit specifies the new output limit.
     * @return return the previous output limit.
     */
    public double saveAndSetOutputLimit(double limit)
    {
        double prevLimit;

        synchronized (pidCtrlState)
        {
            prevLimit = outputLimit;
            outputLimitStack.push(outputLimit);
            setOutputLimit(limit);
        }

        return prevLimit;
    }   //saveAndSetOutputLimit

    /**
     * This method restores the last saved output limit and return its value. If there was no previous call to
     * saveAndSetOutputLimit, the current output limit is returned and the limit is not changed.
     *
     * @return last saved output limit.
     */
    public double restoreOutputLimit()
    {
        double limit;

        synchronized (pidCtrlState)
        {
            try
            {
                limit = outputLimitStack.pop();
                setOutputLimit(limit);
            }
            catch (EmptyStackException e)
            {
                //
                // There was no previous saveAndSetOutputLimit call, don't do anything and just return the current
                // output limit.
                //
                limit = outputLimit;
            }
        }

        return limit;
    }   //restoreOutputLimit

    /**
     * This method returns the position set point value.
     *
     * @return position set point.
     */
    public double getPositionSetpoint()
    {
        return pidCtrlState.posSetPoint;
    }   //getPositionSetpoint

    /**
     * This method returns the velocity set point value.
     *
     * @return velocity set point.
     */
    public double getVelocitySetpoint()
    {
        return pidCtrlState.velSetPoint;
    }   //getVelocitySetpoint

    /**
     * This method returns the acceleration set point value.
     *
     * @return acceleration set point.
     */
    public double getAccelerationSetpoint()
    {
        return pidCtrlState.accelSetPoint;
    }   //getAccelerationSetpoint

    /**
     * This methods sets the target set point.
     *
     * @param target specifies the target set point.
     * @param warpSpace specifies the warp space object if the target is in one, null if not.
     * @param resetError specifies true to reset error state, false otherwise. It is important to preserve error
     *                   state if we are changing target before the PID operation is completed.
     */
    public void setTarget(double target, TrcWarpSpace warpSpace, boolean resetError)
    {
        //
        // Read from input device without holding a lock on this object, since this could
        // be a long-running call.
        // Note: if we are changing target, don't need to waste time to get current input because we are not
        // updating error states anyway.
        //
        final double input = resetError? pidInput.get(): 0.0;

        synchronized (pidCtrlState)
        {
            double error;

            if (resetError)
            {
                pidCtrlState.input = input;
            }

            if (!absSetPoint)
            {
                //
                // Set point is relative, add target to current input to get absolute set point.
                //
                pidCtrlState.posSetPoint = pidCtrlState.input + target;
                error = target;
            }
            else
            {
                //
                // Set point is absolute, use as is but optimize it if it is in warp space.
                //
                pidCtrlState.posSetPoint = target;
                if (warpSpace != null)
                {
                    pidCtrlState.posSetPoint = warpSpace.getOptimizedTarget(pidCtrlState.posSetPoint, pidCtrlState.input);
                }
                error = pidCtrlState.posSetPoint - pidCtrlState.input;
            }

            if (inverted)
            {
                error *= -1.0;
            }

            if (resetError)
            {
                pidCtrlState.posError = error;
                pidCtrlState.velError = 0.0;
                pidCtrlState.totalPosError = 0.0;
                pidCtrlState.setPointSign = Math.signum(error);

                pidCtrlState.timestamp = pidCtrlState.settlingStartTime = TrcTimer.getCurrentTime();
            }
        }
    }   //setTarget

    /**
     * This methods sets the target set point.
     *
     * @param target specifies the target set point.
     * @param warpSpace specifies the warp space object if the target is in one, null if not.
     */
    public void setTarget(double target, TrcWarpSpace warpSpace)
    {
        setTarget(target, warpSpace, true);
    }   //setTarget

    /**
     * This methods sets the target set point.
     *
     * @param target specifies the target set point.
     * @param resetError specifies true to reset error state, false otherwise. It is important to preserve error
     *                   state if we are changing target before the PID operation is completed.
     */
    public void setTarget(double target, boolean resetError)
    {
        setTarget(target, null, resetError);
    }   //setTarget

    /**
     * This methods sets the target set point.
     *
     * @param target specifies the target set point.
     */
    public void setTarget(double target)
    {
        setTarget(target, null, true);
    }   //setTarget

    /**
     * This method returns the error of a previous output calculation.
     *
     * @return previous error.
     */
    public double getError()
    {
        synchronized (pidCtrlState)
        {
            return pidCtrlState.posError;
        }
    }   //getError

    /**
     * This method resets the PID controller clearing the set point, error, total error and output.
     */
    public void reset()
    {
        synchronized (pidCtrlState)
        {
            pidCtrlState.reset();
        }
    }   //reset

    /**
     * This method detects if PID is stalled.
     *
     * @return true if PID is stalled, false otherwise.
     */
    public boolean isStalled()
    {
        boolean stalled = false;

        synchronized (pidCtrlState)
        {
            double currTime = TrcTimer.getCurrentTime();

            if (pidCtrlState.stallDetectionStartTime != null && currTime > pidCtrlState.stallDetectionStartTime)
            {
                if (Math.abs(pidCtrlState.velError) > pidCtrlState.stallErrorRateThreshold)
                {
                    // reset stall start time to current time if it has movement.
                    pidCtrlState.stallDetectionStartTime = currTime;
                }
                else
                {
                    stalled = currTime > pidCtrlState.stallDetectionStartTime + pidCtrlState.stallDetectionTimeout;
                    if (stalled)
                    {
                        tracer.traceInfo(instanceName, "PID stalled.");
                    }
                }
            }
        }

        return stalled;
    }   //isStalled

    public void enableWrapInput(double inputMin, double inputMax)
    {
        synchronized (pidCtrlState)
        {
            pidCtrlState.inputModulus = inputMax - inputMin;
        }
    }   //enableWrapInput

    public void disableWrapInput()
    {
        synchronized (pidCtrlState)
        {
            pidCtrlState.inputModulus = null;
        }
    }   //disableWrapInput

    /**
     * This method determines if we have reached the set point target. It is considered on target if the previous
     * error is smaller than the tolerance and there is no movement for at least settling time. If NoOscillation mode
     * is set, it is considered on target if we are within tolerance or pass target regardless of setting time.
     *
     * @param tolerance specifies the PID error tolerance.
     * @param settlingTime specifies the minimum on target settling time.
     * @return true if we reached target, false otherwise.
     */
    public boolean isOnTarget(double tolerance, double settlingTime)
    {
        boolean onTarget = false;

        synchronized (pidCtrlState)
        {
            double currTime = TrcTimer.getCurrentTime();
            double absErr = Math.abs(pidCtrlState.posError);

            if (noOscillation)
            {
                //
                // Don't allow oscillation, so if we are within tolerance or we pass target, just quit.
                // If setPointSign is positive, it means the target is "forward". So if currError <= tolerance,
                // it means we are either within tolerance or have passed the target.
                // If setPointSign is negative, it means the target is "backward". So if -currError <= tolerance,
                // it means we are either within tolerance or have passed the target.
                //
                if (pidCtrlState.posError*pidCtrlState.setPointSign <= tolerance)
                {
                    onTarget = true;
                }
            }
            //
            // We consider it on-target if error is within tolerance for the settling period.
            //
            else if (absErr > tolerance)
            {
                pidCtrlState.settlingStartTime = TrcTimer.getCurrentTime();
                tracer.traceDebug(
                    instanceName, "InProgress: err=%f, errRate=%f, tolerance=%f",
                    pidCtrlState.posError, pidCtrlState.velError, tolerance);
            }
            else if (currTime >= pidCtrlState.settlingStartTime + settlingTime)
            {
                tracer.traceDebug(
                    instanceName, "OnTarget: err=%f, errRate=%f, tolerance=%f",
                    pidCtrlState.posError, pidCtrlState.velError, tolerance);
                onTarget = true;
            }
        }

        return onTarget;
    }   //isOnTarget

    /**
     * This method determines if we have reached the set point target. It is considered on target if the previous
     * error is smaller than the tolerance and there is no movement for at least settling time. If NoOscillation mode
     * is set, it is considered on target if we are within tolerance or pass target regardless of setting time.
     *
     * @param tolerance specifies the tolerance.
     * @return true if we reached target, false otherwise.
     */
    public boolean isOnTarget(double tolerance)
    {
        return isOnTarget(tolerance, DEF_SETTLING_TIME);
    }   //isOnTarget

    /**
     * This method returns the current PID input value.
     *
     * @return current PID input value.
     */
    public double getCurrentInput()
    {
        return pidInput != null? pidInput.get(): pidCtrlState.input;
    }   //getCurrentInput

   /**
     * This method calculates the PID output applying the PID equation to the given set point targets and current
     * input value.
     *
     * @param input specifies the current input value from the feedback device.
     * @param posSetPoint specifies the position setpoint.
     * @param velSetPoint specifies the velocity setpoint.
     * @param accelSetPoint specifies the acceleration setpoint.
     * @return calculated PID output value.
     */
    public double calculate(double input, Double posSetPoint, Double velSetPoint, Double accelSetPoint)
    {
        /*
        PidCoefficients pidCoeffs;

        double input = 0.0;
        double posSetPoint = 0.0;
        Double velSetPoint = 0.0;
        Double accelSetPoint = 0.0;

        double timestamp = 0.0;
        double deltaTime = 0.0;
        double posError = 0.0;
        double velError = 0.0;
        double totalPosError = 0.0;
         */
        synchronized (pidCtrlState)
        {
            Double prevTimestamp = pidCtrlState.timestamp;
            double prevPosError = pidCtrlState.posError;

            pidCtrlState.input = input;
            if (posSetPoint != null)
            {
                pidCtrlState.posSetPoint = posSetPoint;
                pidCtrlState.velSetPoint = velSetPoint;
                pidCtrlState.accelSetPoint = accelSetPoint;
            }

            pidCtrlState.timestamp = TrcTimer.getCurrentTime();
            pidCtrlState.deltaTime = prevTimestamp != null? pidCtrlState.timestamp - prevTimestamp: 0.0;

            pidCtrlState.posError = inverted? input - posSetPoint: posSetPoint - input;
            // For wrap input, calculate shortest route to setPoint.
            if (pidCtrlState.inputModulus != null)
            {
                double inputBound = pidCtrlState.inputModulus / 2.0;
                pidCtrlState.posError = inputMod(
                    pidCtrlState.posError, -inputBound, inputBound, pidCtrlState.inputModulus);
            }

            pidCtrlState.velError =
                pidCtrlState.deltaTime > 0.0? (pidCtrlState.posError - prevPosError)/pidCtrlState.deltaTime: 0.0;

            // Only allow integration if error is within iZone.
            double absPosError = Math.abs(pidCtrlState.posError);
            if (pidCtrlState.pidCoeffs.kI != 0.0 &&
                (pidCtrlState.pidCoeffs.iZone == 0.0 || absPosError <= pidCtrlState.pidCoeffs.iZone))
            {
                //
                // Make sure the total error doesn't get wound up too much exceeding the Integral range.
                // This is essentially capping the I-term to within the range of minIntegral and maxIntegral.
                //
                pidCtrlState.totalPosError = TrcUtil.clipRange(
                    pidCtrlState.totalPosError + pidCtrlState.posError * pidCtrlState.deltaTime,
                    minIntegral / pidCtrlState.pidCoeffs.kI, maxIntegral / pidCtrlState.pidCoeffs.kI);
            }
            else
            {
                pidCtrlState.totalPosError = 0.0;
            }

            pidCtrlState.pTerm = pidCtrlState.pidCoeffs.kP * pidCtrlState.posError;
            pidCtrlState.iTerm = pidCtrlState.pidCoeffs.kI * pidCtrlState.totalPosError;
            pidCtrlState.dTerm = pidCtrlState.pidCoeffs.kD * pidCtrlState.velError;
            pidCtrlState.fTerm = pidCtrlState.pidCoeffs.kF * pidCtrlState.posSetPoint;
            pidCtrlState.sTerm = pidCtrlState.pidCoeffs.kS * Math.signum(velSetPoint);
            pidCtrlState.vTerm = velSetPoint != null? pidCtrlState.pidCoeffs.kV * velSetPoint: 0.0;
            pidCtrlState.aTerm = accelSetPoint != null? pidCtrlState.pidCoeffs.kA * accelSetPoint: 0.0;

            double output = TrcUtil.clipRange(
                pidCtrlState.pTerm + pidCtrlState.iTerm + pidCtrlState.dTerm +
                pidCtrlState.fTerm + pidCtrlState.sTerm + pidCtrlState.vTerm + pidCtrlState.aTerm,
                minOutput, maxOutput);

            if (rampRate != null)
            {
                double maxChange = rampRate * pidCtrlState.deltaTime;
                double change = TrcUtil.clipRange(output - pidCtrlState.output, -maxChange, maxChange);
                output = pidCtrlState.output + change;
            }

            pidCtrlState.output = output;

            if (tracer.isMsgLevelEnabled(MsgLevel.DEBUG))
            {
                printPidInfo(tracer, pidCtrlState.verbosePidInfo, pidCtrlState.battery);
            }

            return pidCtrlState.output;
        }
    }   //calculate

   /**
     * This method calculates the PID output applying the PID equation to the given set point targets and current
     * input value.
     *
     * @param input specifies the current input value from the feedback device.
     * @param posSetPoint specifies the position setpoint.
     * @param velSetPoint specifies the velocity setpoint.
     * @param accelSetPoint specifies the acceleration setpoint.
     * @return calculated PID output value.
     */
    public double calculate(double input)
    {
        return calculate(input, null, null, null);
    }   //calculate

    private double inputMod(double input, double lowerBound, double upperBound, double modulus)
    {
        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - lowerBound) / modulus);
        input -= numMax * pidCtrlState.inputModulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - upperBound) / modulus);
        input -= numMin * modulus;

        return input;
    }   //inputMod

    /**
     * This method calculates the PID output applying the PID equation to the given set point target and current
     * input value.
     *
     * @return PID output value.
     */
    public double calculate()
    {
        return calculate(pidInput.get(), pidCtrlState.posSetPoint, null, null);
    }   //calculate

    /**
     * This method displays the PID information on the dashboard for debugging and tuning purpose. Note that the
     * PID info occupies two dashboard lines.
     *
     * @param lineNum specifies the starting line number of the dashboard to display the info.
     */
    public void displayPidInfo(int lineNum)
    {
        synchronized (pidCtrlState)
        {
            dashboard.displayPrintf(
                lineNum, "%s:Target=%.1f,Input=%.1f,Error=%.1f",
                instanceName, pidCtrlState.posSetPoint, pidCtrlState.input, pidCtrlState.posError);
            dashboard.displayPrintf(
                lineNum + 1, "minOutput=%.1f,Output=%.1f,maxOutput=%.1f",
                minOutput, pidCtrlState.output, maxOutput);
        }
    }   //displayPidInfo

    /**
     * This method prints the PID information to the tracer console. If no tracer is provided, it will attempt to
     * use the debug tracer in this module but if the debug tracer is not enabled, no output will be produced.
     *
     * @param msgTracer specifies the tracer object to print the PID info to.
     * @param verbose specifies true to print verbose info, false to print summary info.
     * @param battery specifies the battery object to get battery info, can be null if not provided.
     */
    public void printPidInfo(TrcDbgTrace msgTracer, boolean verbose, TrcRobotBattery battery)
    {
        StringBuilder sb = new StringBuilder();

        if (msgTracer == null)
        {
            msgTracer = tracer;
        }

        synchronized (pidCtrlState)
        {
            if (verbose)
            {
                if (battery != null)
                {
                    msgTracer.traceInfo(
                        instanceName,
                        "Target=%6.1f, Input=%6.1f, dT=%.6f, CurrErr=%6.1f, ErrRate=%6.1f, " +
                        "Output=%6.3f(%6.3f/%6.3f), PIDFTerms=%6.3f/%6.3f/%6.3f/%6.3f, Volt=%.1f(%.1f)",
                        pidCtrlState.posSetPoint, pidCtrlState.input, pidCtrlState.deltaTime, pidCtrlState.posError,
                        pidCtrlState.velError, pidCtrlState.output, minOutput, maxOutput,
                        pidCtrlState.pTerm, pidCtrlState.iTerm, pidCtrlState.dTerm, pidCtrlState.fTerm,
                        battery.getVoltage(), battery.getLowestVoltage());
                }
                else
                {
                    tracer.traceInfo(
                        instanceName,
                        "Target=%6.1f, Input=%6.1f, dT=%.6f, CurrErr=%6.1f, ErrRate=%6.1f, " +
                        "Output=%6.3f(%6.3f/%6.3f), PIDFTerms=%6.3f/%6.3f/%6.3f/%6.3f",
                        pidCtrlState.posSetPoint, pidCtrlState.input, pidCtrlState.deltaTime, pidCtrlState.posError,
                        pidCtrlState.velError, pidCtrlState.output, minOutput, maxOutput,
                        pidCtrlState.pTerm, pidCtrlState.iTerm, pidCtrlState.dTerm, pidCtrlState.fTerm);
                }
            }
            else
            {
                if (battery != null)
                {
                    tracer.traceInfo(
                        instanceName,
                        "Target=%6.1f, Input=%6.1f, dT=%.6f, CurrErr=%6.1f, ErrRate=%6.1f, " +
                        "Output=%6.3f(%6.3f/%6.3f), Volt=%.1f(%.1f)",
                        pidCtrlState.posSetPoint, pidCtrlState.input, pidCtrlState.deltaTime, pidCtrlState.posError,
                        pidCtrlState.velError, pidCtrlState.output, minOutput, maxOutput,
                        battery.getVoltage(), battery.getLowestVoltage());
                }
                else
                {
                    tracer.traceInfo(
                        instanceName,
                        "Target=%6.1f, Input=%6.1f, dT=%.6f, CurrErr=%6.1f, ErrRate=%6.1f, " +
                        "Output=%6.3f(%6.3f/%6.3f)",
                        pidCtrlState.posSetPoint, pidCtrlState.input, pidCtrlState.deltaTime, pidCtrlState.posError,
                        pidCtrlState.velError, pidCtrlState.output, minOutput, maxOutput);
                }
            }
        }

        msgTracer.traceInfo(instanceName, sb.toString());
    }   //printPidInfo

    /**
     * This method prints the PID information to the tracer console. If no tracer is provided, it will attempt to
     * use the debug tracer in this module but if the debug tracer is not enabled, no output will be produced.
     *
     * @param tracer specifies the tracer object to print the PID info to.
     * @param verbose specifies true to print verbose info, false to print summary info.
     */
    public void printPidInfo(TrcDbgTrace tracer, boolean verbose)
    {
        printPidInfo(tracer, verbose, null);
    }   //printPidInfo

    /**
     * This method prints the PID information to the tracer console. If no tracer is provided, it will attempt to
     * use the debug tracer in this module but if the debug tracer is not enabled, no output will be produced.
     *
     * @param tracer specifies the tracer object to print the PID info to.
     */
    public void printPidInfo(TrcDbgTrace tracer)
    {
        printPidInfo(tracer, false, null);
    }   //printPidInfo

    /**
     * This method prints the PID information to the default debug tracer.
     */
    public void printPidInfo()
    {
        printPidInfo(null, false, null);
    }   //printPidInfo

}   //class TrcPidController
