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

import trclib.dataprocessor.TrcUtil;
import trclib.dataprocessor.TrcWarpSpace;
import trclib.driverio.TrcDashboard;
import trclib.robotcore.TrcDbgTrace;
import trclib.sensor.TrcRobotBattery;
import trclib.timer.TrcTimer;

/**
 * This class implements a PIDF controller with FeedForward. A PIDF controller takes a target set point and an input
 * from a feedback device to calculate the output power of an effector usually a motor or a set of motors. It also
 * supports Feed Forward if provided. Feed Forward control does not rely on any feedback devices, it derives its
 * output from velocity and/or acceleration set points.
 */
public class TrcPidController
{
    public static final double DEF_SETTLING_TIME = 0.2;
    private static final double DEF_STALL_DETECTION_DELAY = 0.5;
    private static final double DEF_STALL_DETECTION_TIMEOUT = 0.2;
    private static final double DEF_STALL_ERR_RATE_THRESHOLD = 0.1;

    /**
     * This class encapsulates all the PIDF coefficients into a single object and makes it more efficient to pass
     * them around. All coefficients are declared final, so they cannot change once created. If you need to change
     * any coefficient values, you must create a new instance.
     */
    public static class PidCoefficients
    {
        public double kP;
        public double kI;
        public double kD;
        public double kF;
        public double iZone;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional coefficient.
         * @param kI specifies the Integral coefficient.
         * @param kD specifies the Differential coefficient.
         * @param kF specifies the Feed forward coefficient.
         * @param iZone specifies the maximum magnitude of error to allow integral control.
         */
        public PidCoefficients(double kP, double kI, double kD, double kF, double iZone)
        {
            this.kP = Math.abs(kP);
            this.kI = Math.abs(kI);
            this.kD = Math.abs(kD);
            this.kF = Math.abs(kF);
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
            this(kP, kI, kD, kF, 0.0);
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
            this(kP, kI, kD, 0.0, 0.0);
        }   //PidCoefficients

        /**
         * This method returns all PID coefficients in string form.
         *
         * @return PID coefficients string.
         */
        @Override
        public String toString()
        {
            return "(PIDFZ:" + kP + "," + kI + "," + kD + "," + kF + "," + iZone + ")";
        }   //toString

    }   //class PidCoefficients

    /**
     * This class encapsulates all the FeedForward coefficients into a single object and makes it more efficient to
     * pass them around. All coefficients are declared final, so they cannot change once created. If you need to change
     * any coefficient values, you must create a new instance.
     */
    public static class FFCoefficients
    {
        public double kS;
        public double kV;
        public double kA;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kS specifies the Static gain.
         * @param kV specifies the Velocity gain.
         * @param kA specifies the Acceleration gain.
         */
        public FFCoefficients(double kS, double kV, double kA)
        {
            this.kS = kS;
            this.kV = Math.abs(kV);
            this.kA = Math.abs(kA);
        }   //FFCoefficients

        /**
         * This method returns all PID coefficients in string form.
         *
         * @return PID coefficients string.
         */
        @Override
        public String toString()
        {
            return "(SVA:" + kS + "," + kV + "," + kA + ")";
        }   //toString

    }   //class FFCoefficients

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
        FFCoefficients ffCoeffs = null;
        Double inputMin = null;
        Double inputMax = null;

        double input = 0.0;
        double posSetpoint = 0.0;
        double velSetpoint = 0.0;
        double accelSetpoint = 0.0;

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
            posSetpoint = 0.0;
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
    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final PidInput pidInput;

    private boolean inverted = false;
    private boolean absSetPoint = false;
    private boolean noOscillation = false;
    private boolean squareRootOutput = false;
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
     * @param pidCoeffs specifies the PIDF coefficients.
     * @param ffCoeffs specifies the FeedForward coefficients.
     * @param pidInput specifies the method to call for getting input.
     */
    public TrcPidController(String instanceName, PidCoefficients pidCoeffs, FFCoefficients ffCoeffs, PidInput pidInput)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.pidInput = pidInput;
        synchronized (pidCtrlState)
        {
            pidCtrlState.pidCoeffs = pidCoeffs;
            pidCtrlState.ffCoeffs = ffCoeffs;
        }
    }   //TrcPidController

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pidCoeffs specifies the PID coefficients.
     * @param pidInput specifies the method to call to get PID sensor input.
     */
    public TrcPidController(String instanceName, PidCoefficients pidCoeffs, PidInput pidInput)
    {
        this(instanceName, pidCoeffs, null, pidInput);
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
     * This method enables/disables the mode that square rooting the PID output. By square rooting the PID output,
     * it gives a boost to the output when the error is smaller. That means it will make PID stronger to reach
     * target. Apparently, this strategy is widely used in Washington state teams and rumored that it came from
     * the team Escape Velocity.
     *
     * @param enable specifies true to enable and false to disable.
     */
    public void setSquareRootOutputEnabled(boolean enable)
    {
        synchronized (pidCtrlState)
        {
            this.squareRootOutput = enable;
        }
    }   //setSquareRootOutputEnabled

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
     * This method returns the current FF coefficients.
     *
     * @return current FF coefficients.
     */
    public FFCoefficients getFFCoefficients()
    {
        synchronized (pidCtrlState)
        {
            return pidCtrlState.ffCoeffs;
        }
    }   //getFFCoefficients

    /**
     * This method sets new FF coefficients.
     *
     * @param ffCoeffs specifies new FF coefficients.
     */
    public void setFFCoefficients(FFCoefficients ffCoeffs)
    {
        synchronized (pidCtrlState)
        {
            pidCtrlState.ffCoeffs = ffCoeffs;
        }
    }   //setFFCoefficients

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
        return pidCtrlState.posSetpoint;
    }   //getPositionSetpoint

    /**
     * This method returns the velocity set point value.
     *
     * @return velocity set point.
     */
    public double getVelocitySetpoint()
    {
        return pidCtrlState.velSetpoint;
    }   //getVelocitySetpoint

    /**
     * This method returns the acceleration set point value.
     *
     * @return acceleration set point.
     */
    public double getAccelerationSetpoint()
    {
        return pidCtrlState.accelSetpoint;
    }   //getAccelerationSetpoint

    /**
     * This methods sets the target set point.
     *
     * @param target specifies the target set point.
     * @param warpSpace specifies the warp space object if the target is in one, null if not.
     */
    public void setTarget(double target, TrcWarpSpace warpSpace)
    {
        // Read from input device without holding a lock on this object, since this could be a long-running call.
        final double input = pidInput.get();

        synchronized (pidCtrlState)
        {
            double error;

            pidCtrlState.input = input;
            if (!absSetPoint)
            {
                //
                // Set point is relative, add target to current input to get absolute set point.
                //
                pidCtrlState.posSetpoint = pidCtrlState.input + target;
                error = target;
            }
            else
            {
                //
                // Set point is absolute, use as is but optimize it if it is in warp space.
                //
                pidCtrlState.posSetpoint = target;
                if (warpSpace != null)
                {
                    pidCtrlState.posSetpoint = warpSpace.getOptimizedTarget(
                        pidCtrlState.posSetpoint, pidCtrlState.input);
                }
                error = pidCtrlState.posSetpoint - pidCtrlState.input;
            }

            if (inverted)
            {
                error *= -1.0;
            }

            pidCtrlState.velSetpoint = 0.0;
            pidCtrlState.accelSetpoint = 0.0;
            pidCtrlState.posError = error;
            pidCtrlState.velError = 0.0;
            pidCtrlState.totalPosError = 0.0;
            pidCtrlState.setPointSign = Math.signum(error);
            pidCtrlState.timestamp = pidCtrlState.settlingStartTime = TrcTimer.getCurrentTime();
        }
    }   //setTarget

    /**
     * This methods sets the target set point.
     *
     * @param target specifies the target set point.
     */
    public void setTarget(double target)
    {
        setTarget(target, null);
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
     * This method returns the error rate.
     *
     * @return error rate.
     */
    public double getErrorRate()
    {
        synchronized (pidCtrlState)
        {
            return pidCtrlState.velError;
        }
    }   //getErrorRate

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
                        tracer.traceInfo(
                            instanceName,
                            "PID stalled (stallTime=" + (currTime - pidCtrlState.stallDetectionStartTime) +
                            "/" + pidCtrlState.stallDetectionTimeout +
                            ", error=" + pidCtrlState.posError +
                            ", errorRate=" + pidCtrlState.velError +
                            "/" + pidCtrlState.stallErrorRateThreshold + ").");
                    }
                }
            }
        }

        return stalled;
    }   //isStalled

    /**
     * This method enables continuous input.
     * <p>Rather then using the max and min input range as constraints, it considers them to be the same point and
     * automatically calculates the shortest route to the setpoint (this is equivalent to TrcWarpSpace).
     *
     * @param inputMin specifies the minimum value of input range.
     * @param inputMax specifies the maximum value of input range.
     */
    public void enableContinuousInput(double inputMin, double inputMax)
    {
        if (inputMin >= inputMax)
        {
            throw new IllegalArgumentException("inputMax must be greater than inputMin.");
        }

        synchronized (pidCtrlState)
        {
            pidCtrlState.inputMin = inputMin;
            pidCtrlState.inputMax = inputMax;
        }
    }   //enableContinuousInput

    /**
     * This method disables continuous input.
     */
    public void disableContinuousInput()
    {
        synchronized (pidCtrlState)
        {
            pidCtrlState.inputMin = pidCtrlState.inputMax = null;
        }
    }   //disableContinuousInput

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
                tracer.traceDebug(
                    instanceName,
                    "NoOscillation: err=%f, errRate=%f, tolerance=%f, setPointSign=%f, onTarget=%s",
                    pidCtrlState.posError, pidCtrlState.velError, tolerance, pidCtrlState.setPointSign, onTarget);
            }
            //
            // We consider it on-target if error is within tolerance for the settling period.
            //
            else if (absErr > tolerance)
            {
                pidCtrlState.settlingStartTime = TrcTimer.getCurrentTime();
                tracer.traceDebug(
                    instanceName,
                    "InProgress: err=%f, errRate=%f, tolerance=%f",
                    pidCtrlState.posError, pidCtrlState.velError, tolerance);
           }
            else if (settlingTime == 0.0 || currTime >= pidCtrlState.settlingStartTime + settlingTime)
            {
                tracer.traceDebug(
                    instanceName,
                    "OnTarget: err=%f, errRate=%f, tolerance=%f",
                    pidCtrlState.posError, pidCtrlState.velError, tolerance);
                onTarget = true;
            }
            else
            {
                tracer.traceDebug(
                    instanceName,
                    "Settling: err=%f, errRate=%f, tolerance=%f, currTime=%f, expiredTime=%f",
                    pidCtrlState.posError, pidCtrlState.velError, tolerance, currTime,
                    pidCtrlState.settlingStartTime + settlingTime);
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
    * input value. It also applies FeedForward if provided.
    *
    * @param input specifies the current input value from the feedback device, null if not provided in which case
    *         pidInput is called to get the current input value.
    * @param posSetpoint specifies the position setpoint, null if not provided in which case the previous setpoint is
    *        used unchanged.
    * @param velSetpoint specifies the velocity setpoint, null if not provided in which case the previous setupoint is
    *        used unchanged (could be null).
    * @param accelSetpoint specifies the acceleration setpoint, null if not provided in which case the previous
    *        setpoint is used unchanged (could be null).
    * @return calculated PID output value.
    */
    public double calculate(Double input, Double posSetpoint, Double velSetpoint, Double accelSetpoint)
    {
        if (input == null)
        {
            input = pidInput.get();
        }

        synchronized (pidCtrlState)
        {
            pidCtrlState.input = input;
            // If posSetPoint parameter is null, setpoints were set by setTarget call and have not changed.
            if (posSetpoint != null)
            {
                // Relative setpoint is only applicable for position setpoint, all other setpoints are absolute.
                pidCtrlState.posSetpoint = absSetPoint ? posSetpoint : pidCtrlState.input + posSetpoint;
            }

            if (velSetpoint != null)
            {
                pidCtrlState.velSetpoint = velSetpoint;
            }

            if (accelSetpoint != null)
            {
                pidCtrlState.accelSetpoint = accelSetpoint;
            }
            // Calculate detla time.
            Double prevTimestamp = pidCtrlState.timestamp;
            pidCtrlState.timestamp = TrcTimer.getCurrentTime();
            pidCtrlState.deltaTime = prevTimestamp != null? pidCtrlState.timestamp - prevTimestamp: 0.0;

            // Calculate position error.
            double prevPosError = pidCtrlState.posError;
            pidCtrlState.posError = inverted? input - pidCtrlState.posSetpoint: pidCtrlState.posSetpoint - input;
            // For continuous input, calculate shortest route to setPoint.
            if (pidCtrlState.inputMin != null && pidCtrlState.inputMax != null)
            {
                double errorBound = (pidCtrlState.inputMax - pidCtrlState.inputMin) / 2.0;
                pidCtrlState.posError = inputMod(pidCtrlState.posError, -errorBound, errorBound);
            }
            pidCtrlState.setPointSign = Math.signum(pidCtrlState.posError);

            // Calculate velocity error (differential error).
            // If deltaTime is zero, this is the first call to pid-calculate or the posSetpoint has changed.
            // In this case, we don't have velocity error (aka error rate). We also reset error integral.
            pidCtrlState.velError =
                pidCtrlState.deltaTime > 0.0? (pidCtrlState.posError - prevPosError)/pidCtrlState.deltaTime: 0.0;

            // Calculate integral error.
            // Only allow integration if error is within iZone.
            double absPosError = Math.abs(pidCtrlState.posError);
            if (pidCtrlState.deltaTime > 0.0 && pidCtrlState.pidCoeffs.kI != 0.0 &&
                (pidCtrlState.pidCoeffs.iZone == 0.0 || absPosError <= pidCtrlState.pidCoeffs.iZone))
            {
                // Make sure the total error doesn't get wound up too much exceeding the Integral range.
                // This is essentially capping the I-term to within the range of minIntegral and maxIntegral.
                pidCtrlState.totalPosError = TrcUtil.clipRange(
                    pidCtrlState.totalPosError + pidCtrlState.posError * pidCtrlState.deltaTime,
                    minIntegral / pidCtrlState.pidCoeffs.kI, maxIntegral / pidCtrlState.pidCoeffs.kI);
            }
            else
            {
                pidCtrlState.totalPosError = 0.0;
            }

            // Calculate PIDF output.
            pidCtrlState.pTerm = pidCtrlState.pidCoeffs.kP * pidCtrlState.posError;
            pidCtrlState.iTerm = pidCtrlState.pidCoeffs.kI * pidCtrlState.totalPosError;
            pidCtrlState.dTerm = pidCtrlState.pidCoeffs.kD * pidCtrlState.velError;
            pidCtrlState.fTerm = pidCtrlState.pidCoeffs.kF * pidCtrlState.posSetpoint;
            double output = pidCtrlState.pTerm + pidCtrlState.iTerm + pidCtrlState.dTerm + pidCtrlState.fTerm;

            // Calculate FeedForward if any.
            if (pidCtrlState.ffCoeffs != null)
            {
                pidCtrlState.sTerm = pidCtrlState.ffCoeffs.kS * Math.signum(pidCtrlState.velSetpoint);
                pidCtrlState.vTerm = pidCtrlState.ffCoeffs.kV * pidCtrlState.velSetpoint;
                pidCtrlState.aTerm = pidCtrlState.ffCoeffs.kA * pidCtrlState.accelSetpoint;
                output += pidCtrlState.sTerm + pidCtrlState.vTerm + pidCtrlState.aTerm;
            }
            output = TrcUtil.clipRange(output, minOutput, maxOutput);

            // Apply Ramp Rate limit if any.
            if (rampRate != null)
            {
                double maxChange = rampRate * pidCtrlState.deltaTime;
                double change = TrcUtil.clipRange(output - pidCtrlState.output, -maxChange, maxChange);
                output = pidCtrlState.output + change;
            }

            if (squareRootOutput)
            {
                output = Math.signum(output) * Math.sqrt(Math.abs(output));
            }
            pidCtrlState.output = output;

            if (tracer.isMsgLevelEnabled(TrcDbgTrace.MsgLevel.DEBUG))
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
     * @param posSetpoint specifies the position setpoint.
     * @return calculated PID output value.
     */
    public double calculate(double input, double posSetpoint)
    {
        return calculate(input, posSetpoint, null, null);
    }   //calculate

   /**
     * This method calculates the PID output applying the PID equation to the given set point targets and current
     * input value.
     *
     * @param input specifies the current input value from the feedback device.
     * @return calculated PID output value.
     */
    public double calculate(double input)
    {
        return calculate(input, null, null, null);
    }   //calculate

    /**
     * This method calculates the PID output applying the PID equation to the given set point targets and current
     * input value.
     *
     * @return calculated PID output value.
     */
    public double calculate()
    {
        return calculate(pidInput.get(), null, null, null);
    }   //calculate

    /**
     * This method converts a continuous input value to a wrap value by performing a modulus operation.
     *
     * @param input specifies the input value to wrap.
     * @param lowerBound specifies the lower bound of the wrap value.
     * @param upperBound specifies the upper bound of the wrap value.
     * @return wrapped value.
     */
    private double inputMod(double input, double lowerBound, double upperBound)
    {
        double modulus = upperBound - lowerBound;
        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - lowerBound) / modulus);
        input -= numMax * modulus;
        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - upperBound) / modulus);
        input -= numMin * modulus;

        return input;
    }   //inputMod

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
                lineNum,
                instanceName +
                ":Target=" + pidCtrlState.posSetpoint +
                ",Input=" + pidCtrlState.input +
                ",Error=" + pidCtrlState.posError);
            dashboard.displayPrintf(
                lineNum + 1,
                "minOutput=" + minOutput +
                ",Output=" + pidCtrlState.output +
                ",maxOutput=" + maxOutput);
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
                        "Target=" + pidCtrlState.posSetpoint +
                        ", Input=" + pidCtrlState.input +
                        ", dT=" + pidCtrlState.deltaTime +
                        ", CurrErr=" + pidCtrlState.posError +
                        ", ErrRate=" + pidCtrlState.velError +
                        ", Output=" + pidCtrlState.output + "(" + minOutput + "/" + maxOutput +
                        "), PIDFTerms=" + pidCtrlState.pTerm +
                        "/" + pidCtrlState.iTerm +
                        "/" + pidCtrlState.dTerm +
                        "/" + pidCtrlState.fTerm +
                        ", Volt=" + battery.getVoltage() +
                        "(" + battery.getLowestVoltage() + ")");
                }
                else
                {
                    msgTracer.traceInfo(
                        instanceName,
                        "Target=" + pidCtrlState.posSetpoint +
                        ", Input=" + pidCtrlState.input +
                        ", dT=" + pidCtrlState.deltaTime +
                        ", CurrErr=" + pidCtrlState.posError +
                        ", ErrRate=" + pidCtrlState.velError +
                        ", Output=" + pidCtrlState.output + "(" + minOutput + "/" + maxOutput +
                        "), PIDFTerms=" + pidCtrlState.pTerm +
                        "/" + pidCtrlState.iTerm +
                        "/" + pidCtrlState.dTerm +
                        "/" + pidCtrlState.fTerm);
                }
            }
            else
            {
                if (battery != null)
                {
                    msgTracer.traceInfo(
                        instanceName,
                        "Target=" + pidCtrlState.posSetpoint +
                        ", Input=" + pidCtrlState.input +
                        ", dT=" + pidCtrlState.deltaTime +
                        ", CurrErr=" + pidCtrlState.posError +
                        ", ErrRate=" + pidCtrlState.velError +
                        ", Output=" + pidCtrlState.output + "(" + minOutput + "/" + maxOutput +
                        "), Volt=" + battery.getVoltage() +
                        "(" + battery.getLowestVoltage() + ")");
                }
                else
                {
                    msgTracer.traceInfo(
                        instanceName,
                        "Target=" + pidCtrlState.posSetpoint +
                        ", Input=" + pidCtrlState.input +
                        ", dT=" + pidCtrlState.deltaTime +
                        ", CurrErr=" + pidCtrlState.posError +
                        ", ErrRate=" + pidCtrlState.velError +
                        ", Output=" + pidCtrlState.output + "(" + minOutput + "/" + maxOutput + ")");
                }
            }
        }
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
