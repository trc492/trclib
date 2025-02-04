/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib.motor;

import trclib.robotcore.TrcPidController;

/**
 * This interface implements a generic motor. Some motors natively support some of the methods but simulate others.
 * Some methods may be unsupported by the motor in which case it may throw an UnsupportedOperationException.
 */
public interface TrcMotorController
{
    //
    // The following methods must be provided by platform specific motor controllers because there is no simulation
    // in TrcMotor for these features. If the motor controller does not have support for some of these methods, it
    // should throw an UnsupportedOperationException.
    //

    /**
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    void resetFactoryDefault();

    /**
     * This method returns the bus voltage of the motor controller.
     *
     * @return bus voltage of the motor controller.
     */
    double getBusVoltage();

    /**
     * This method sets the current limit of the motor.
     *
     * @param currentLimit specifies the current limit (holding current) in amperes when feature is activated.
     * @param triggerThresholdCurrent specifies threshold current in amperes to be exceeded before limiting occurs.
     *        If this value is less than currentLimit, then currentLimit is used as the threshold.
     * @param triggerThresholdTime specifies how long current must exceed threshold (seconds) before limiting occurs.
     */
    void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime);

    /**
     * This method sets the stator current limit of the motor.
     *
     * @param currentLimit specifies the stator current limit in amperes.
     */
    void setStatorCurrentLimit(double currentLimit);

    /**
     * This method sets the close loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    void setCloseLoopRampRate(double rampTime);

    /**
     * This method sets the open loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    void setOpenLoopRampRate(double rampTime);

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When not enabled,
     * (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor will
     * stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    void setBrakeModeEnabled(boolean enabled);

    /**
     * This method enables the reverse limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    void enableMotorRevLimitSwitch(boolean normalClose);

    /**
     * This method enables the forward limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    void enableMotorFwdLimitSwitch(boolean normalClose);

    /**
     * This method disables the reverse limit switch.
     */
    void disableMotorRevLimitSwitch();

    /**
     * This method disables the forward limit switch.
     */
    void disableMotorFwdLimitSwitch();

    /**
     * This method checks if the reverse limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    boolean isMotorRevLimitSwitchEnabled();

    /**
     * This method checks if the forward limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    boolean isMotorFwdLimitSwitchEnabled();

    /**
     * This method inverts the active state of the reverse limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch to normal close, false to normal open.
     */
    void setMotorRevLimitSwitchInverted(boolean inverted);

    /**
     * This method inverts the active state of the forward limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch to normal close, false to normal open.
     */
    void setMotorFwdLimitSwitchInverted(boolean inverted);

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    boolean isMotorRevLimitSwitchActive();

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    boolean isMotorFwdLimitSwitchActive();

    /**
     * This method sets the soft position limit for the reverse direction.
     *
     * @param limit specifies the limit in sensor units, null to disable.
     */
    void setMotorRevSoftPositionLimit(Double limit);

    /**
     * This method sets the soft position limit for the forward direction.
     *
     * @param limit specifies the limit in sensor units, null to disable.
     */
    void setMotorFwdSoftPositionLimit(Double limit);

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    void setMotorPositionSensorInverted(boolean inverted);

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    boolean isMotorPositionSensorInverted();

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    void resetMotorPosition();

    /**
     * This method inverts the spinning direction of the motor.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    void setMotorInverted(boolean inverted);

    /**
     * This method checks if the motor direction is inverted.
     *
     * @return true if motor direction is inverted, false otherwise.
     */
    boolean isMotorInverted();

    /**
     * This method sets the percentage motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0).
     */
    void setMotorPower(double power);

    /**
     * This method gets the current motor power.
     *
     * @return current motor power.
     */
    double getMotorPower();

    /**
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param velocity specifies the motor velocity in rotations per second.
     * @param acceleration specifies the max motor acceleration rotations per second square, can be 0 if not provided.
     * @param feedForward specifies feedforward in volts.
     */
    void setMotorVelocity(double velocity, double acceleration, double feedForward);

    /**
     * This method returns the current motor velocity.
     *
     * @return current motor velocity in raw sensor units per sec.
     */
    double getMotorVelocity();

    /**
     * This method commands the motor to go to the given position using close loop control and optionally limits the
     * power of the motor movement.
     *
     * @param position specifies the position in rotations.
     * @param powerLimit specifies the maximum power output limits, can be null if not provided. If not provided, the
     *        previous set limit is applied.
     * @param velocity specifies the max motor velocity rotations per second, can be 0 if not provided.
     * @param feedForward specifies feedforward in volts.
     */
    void setMotorPosition(double position, Double powerLimit, double velocity, double feedForward);

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position in sensor units.
     */
    double getMotorPosition();

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    void setMotorCurrent(double current);

    /**
     * This method returns the motor current.
     *
     * @return motor current in amperes.
     */
    double getMotorCurrent();

    /**
     * This method sets the PID coefficients of the motor controller's velocity PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    void setMotorVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoeff);

    /**
     * This method returns the PID coefficients of the motor controller's velocity PID controller.
     *
     * @return PID coefficients of the motor's veloicty PID controller.
     */
    TrcPidController.PidCoefficients getMotorVelocityPidCoefficients();

    /**
     * This method sets the PID coefficients of the motor controller's position PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    void setMotorPositionPidCoefficients(TrcPidController.PidCoefficients pidCoeff);

    /**
     * This method returns the PID coefficients of the motor controller's position PID controller.
     *
     * @return PID coefficients of the motor's position PID controller.
     */
    TrcPidController.PidCoefficients getMotorPositionPidCoefficients();

    /**
     * This method sets the PID coefficients of the motor controller's current PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    void setMotorCurrentPidCoefficients(TrcPidController.PidCoefficients pidCoeff);

    /**
     * This method returns the PID coefficients of the motor controller's current PID controller.
     *
     * @return PID coefficients of the motor's current PID controller.
     */
    TrcPidController.PidCoefficients getMotorCurrentPidCoefficients();

    //
    // The following methods simulate features that the motor controller does not have support for. If the motor
    // controller does support any of these features, it should override these methods and provide direct support
    // in hardware.
    //

    /**
     * This method enables/disables voltage compensation so that it will maintain the motor output regardless of
     * battery voltage.
     *
     * @param batteryNominalVoltage specifies the nominal voltage of the battery to enable, null to disable.
     */
    void setVoltageCompensationEnabled(Double batteryNominalVoltage);

    /**
     * This method checks if voltage compensation is enabled.
     *
     * @return true if voltage compensation is enabled, false if disabled.
     */
    boolean isVoltageCompensationEnabled();

    /**
     * This method enables motion profile support.
     *
     * @param velocity specifies cruise velocity in the unit of rps.
     * @param acceleration specifies acceleration in the unit of rot per sec square.
     * @param deceleration specifies deceleration in the unit of rot per sec square.
     * @param jerk specifies acceleration derivation in the unit of rot per sec cube.
     */
    void enableMotionProfile(double velocity, double acceleration, double deceleration, double jerk);

    /**
     * This method disables motion profile support.
     */
    void disableMotionProfile();

    /**
     * This method sets this motor to follow another motor. Motor subclass should override this if it supports this
     * natively. TrcMotor by default supports software follower list.
     *
     * @param motor specifies the motor to follow.
     * @param inverted specifies true if this motor is inverted from the motor it is following, false otherwise.
     * @param scale specifies the value scale for the follower motor, 1.0 by default.
     */
    void follow(TrcMotor motor, boolean inverted, double scale);

    /**
     * This method returns the follower with the specified index.
     *
     * @param index specifies the follower index.
     * @return follower.
     */
    TrcMotor getFollower(int index);

}   //interface TrcMotorController
