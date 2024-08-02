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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent auto-assist shooter subsystem. It consists of a shooter motor and
 * optionally a tilt motor and/or a pan motor. It provides methods to automate the shooting operation which includes
 * aiming by panning and tilting to the specified angles and spinning the shooter motor to the specified velocity.
 * It then uses the caller provided shoot method to shoot the object and signals completion if necessary.
 */
public class TrcShooter implements TrcExclusiveSubsystem
{
    /**
     * This interface must be implemented by the caller to provide a method for shooting the object.
     */
    public interface ShootOperation
    {
        void shoot(String owner, TrcEvent completionEvent);
    }   //interface ShootOperation

    /**
     * This class encapsulates the parameters for the pan or tilt motors.
     */
    public static class PanTiltParams
    {
        private double powerLimit;
        private double minPos, maxPos;

        public PanTiltParams(double powerLimit, double minPos, double maxPos)
        {
            this.powerLimit = powerLimit;
            this.minPos = minPos;
            this.maxPos = maxPos;
        }   //PanTiltrParams

    }   //class PanTiltParams

    public final TrcDbgTrace tracer;
    private final String instanceName;
    public final TrcMotor shooterMotor1;
    public final TrcMotor shooterMotor2;
    public final TrcMotor tiltMotor;
    private final PanTiltParams tiltParams;
    public final TrcMotor panMotor;
    private final PanTiltParams panParams;
    private final TrcTimer aimTimer;
    private final TrcTimer shootTimer;

    private String currOwner = null;
    private TrcEvent completionEvent = null;
    private ShootOperation shootOp = null;
    private String shootOpOwner = null;
    private Double shootOffDelay = null;
    private boolean active = false;
    private TrcEvent shooter1OnTargetEvent = null;
    private TrcEvent shooter2OnTargetEvent = null;
    private TrcEvent tiltOnTargetEvent = null;
    private TrcEvent panOnTargetEvent = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param shooterMotor1 specifies the shooter motor 1 object.
     * @param shooterMotor2 specifies the shooter motor 2 object, can be null for one-motor shooter.
     * @param tiltMotor specifies the tilt motor object, can be null if none.
     * @param tiltParams specifies the tilt parameters, null if no tilt motor.
     * @param panMotor specifies the pan motor object, can be null if none.
     * @param panParams specifies the pan parameters, null if no pan motor.
     */
    public TrcShooter(
        String instanceName, TrcMotor shooterMotor1, TrcMotor shooterMotor2,
        TrcMotor tiltMotor, PanTiltParams tiltParams, TrcMotor panMotor, PanTiltParams panParams)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;
        this.tiltMotor = tiltMotor;
        this.tiltParams = tiltParams;
        this.panMotor = panMotor;
        this.panParams = panParams;

        aimTimer = new TrcTimer(instanceName + ".aimTimer");
        shootTimer = new TrcTimer(instanceName + ".shootTimer");
    }   //TrcShooter

    /**
     * This method checks if the shooter is active.
     *
     * @return true if shooter is active, false otherwise.
     */
    public boolean isActive()
    {
        return active;
    }   //isActive

    /**
     * This method is called when the shooter operation is finished or canceled.
     *
     * @param completed specifies true if the operation is completed, false if canceled.
     */
    private void finish(boolean completed)
    {
        aimTimer.cancel();
        shootTimer.cancel();

        if (!completed)
        {
            // The operation was canceled, stop the shooter motor.
            stopShooter();
        }
        shootOp = null;
        shootOpOwner = null;
        shootOffDelay = null;

        if (tiltMotor != null)
        {
            tiltMotor.stop();
        }

        if (panMotor != null)
        {
            panMotor.stop();
        }

        if (currOwner != null)
        {
            releaseExclusiveAccess(currOwner);
            currOwner = null;
        }

        if (completionEvent != null)
        {
            if (completed)
            {
                completionEvent.signal();
            }
            else
            {
                completionEvent.cancel();
            }
            completionEvent = null;
        }

        active = false;
    }   //finish

    /**
     * This method cancel a pending shooter operation if any.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
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
     * This method cancel a pending shooter operation if any.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method sets the shooter velocity and the tilt/pan angles if tilt/pan exist. This method is asynchronous.
     * When both shooter velocity and tilt/pan positions have reached target and if shoot method is provided, it will
     * shoot and signal an event if provided.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param velocity1 specifies the shooter motor 1 velocity in revolutions per second.
     * @param velocity2 specifies the shooter motor 2 velocity in revolutions per second, ignored if none.
     * @param tiltAngle specifies the absolute tilt angle in degrees.
     * @param panAngle specifies the absolute pan angle in degrees.
     * @param event specifies an event to signal when both reached target, can be null if not provided.
     * @param timeout specifies maximum timeout period, can be zero if no timeout.
     * @param shootOp specifies the shoot method, can be null if aim only.
     * @param shootOffDelay specifies the delay in seconds to turn off shooter after shooting, or zero if no delay
     *        (turn off immediately), only applicable if shootOp is not null. Can also be null if keeping the shooter
     *        on.
     */
    public void aimShooter(
        String owner, double velocity1, double velocity2, double tiltAngle, double panAngle, TrcEvent event,
        double timeout, ShootOperation shootOp, Double shootOffDelay)
    {
        tracer.traceDebug(
            instanceName,
            "owner=" + owner +
            ", currOwner=" + getCurrentOwner() +
            ", vel=" + velocity1 + "/" + velocity2 +
            ", tiltAngle=" + tiltAngle +
            ", panAngle=" + panAngle +
            ", event=" + event +
            ", timeout=" + timeout +
            ", aimOnly=" + (shootOp == null) +
            ", shootOffDelay=" + shootOffDelay);
        // Caller specifies an owner but has not acquired ownership, let's acquire ownership on its behalf.
        if (owner != null && !hasOwnership(owner) && acquireExclusiveAccess(owner))
        {
            currOwner = owner;
        }

        if (validateOwnership(owner))
        {
            this.completionEvent = event;
            this.shootOp = shootOp;
            this.shootOpOwner = shootOp != null? owner: null;
            this.shootOffDelay = shootOffDelay;

            shooter1OnTargetEvent = new TrcEvent(instanceName + ".shooter1OnTarget");
            shooter1OnTargetEvent.setCallback(this::onTarget, null);
            shooterMotor1.setVelocity(0.0, velocity1, 0.0, shooter1OnTargetEvent);
            if (shooterMotor2 != null)
            {
                shooter2OnTargetEvent = new TrcEvent(instanceName + ".shooter2OnTarget");
                shooter2OnTargetEvent.setCallback(this::onTarget, null);
                shooterMotor2.setVelocity(0.0, velocity2, 0.0, shooter2OnTargetEvent);
            }

            if (tiltMotor != null)
            {
                tiltOnTargetEvent = new TrcEvent(instanceName + ".tiltOnTarget");
                tiltOnTargetEvent.setCallback(this::onTarget, null);
                tiltMotor.setPosition(0.0, tiltAngle, true, tiltParams.powerLimit, tiltOnTargetEvent);
            }

            if (panMotor != null)
            {
                panOnTargetEvent = new TrcEvent(instanceName + ".panOnTarget");
                panOnTargetEvent.setCallback(this::onTarget, null);
                panMotor.setPosition(0.0, panAngle, true, panParams.powerLimit, panOnTargetEvent);
            }

            if (timeout > 0.0)
            {
                aimTimer.set(timeout, this::timedOut, false);
            }

            active = true;
        }
    }   //aimShooter

    /**
     * This method sets the shooter velocity and the tilt/pan angles if tilt/pan exist. This method is asynchronous.
     * When both shooter velocity and tilt/pan positions have reached target and if shoot method is provided, it will
     * shoot and signal an event if provided.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param velocity1 specifies the shooter motor 1 velocity in revolutions per second.
     * @param velocity2 specifies the shooter motor 2 velocity in revolutions per second, ignored if none.
     * @param tiltAngle specifies the absolute tilt angle in degrees.
     * @param panAngle specifies the absolute pan angle in degrees.
     * @param event specifies an event to signal when both reached target, can be null if not provided.
     * @param timeout specifies maximum timeout period, can be zero if no timeout.
     */
    public void aimShooter(
        String owner, double velocity1, double velocity2, double tiltAngle, double panAngle, TrcEvent event,
        double timeout)
    {
        aimShooter(owner, velocity1, velocity2, tiltAngle, panAngle, event, timeout, null, null);
    }   //aimShooter

    /**
     * This method sets the shooter velocity and the tilt/pan angles if tilt/pan exist. This method is asynchronous.
     * When both shooter velocity and tilt/pan positions have reached target and if shoot method is provided, it will
     * shoot and signal an event if provided.
     *
     * @param velocity1 specifies the shooter motor 1 velocity in revolutions per second.
     * @param velocity2 specifies the shooter motor 2 velocity in revolutions per second, ignored if none.
     * @param tiltAngle specifies the absolute tilt angle in degrees.
     * @param panAngle specifies the absolute pan angle in degrees.
     */
    public void aimShooter(double velocity1, double velocity2, double tiltAngle, double panAngle)
    {
        aimShooter(null, velocity1, velocity2, tiltAngle, panAngle, null, 0.0, null, null);
    }   //aimShooter

    /**
     * This method is called when the shooter has reached target velocity or tilt/pan has reached target positions.
     *
     * @param context not used.
     */
    private void onTarget(Object context)
    {
        tracer.traceDebug(
            instanceName,
            "shooter1Event=" + shooter1OnTargetEvent +
            ",shooter2Event=" + shooter2OnTargetEvent +
            ", tiltEvent=" + tiltOnTargetEvent +
            ", panEvent=" + panOnTargetEvent +
            ", aimOnly=" + (shootOp == null));
        if (shooter1OnTargetEvent.isSignaled() &&
            (shooter2OnTargetEvent == null || shooter2OnTargetEvent.isSignaled()) &&
            (tiltOnTargetEvent == null || tiltOnTargetEvent.isSignaled()) &&
            (panOnTargetEvent == null || panOnTargetEvent.isSignaled()))
        {
            if (shootOp != null)
            {
                // If both shooter velocity and tilt/pan position have reached target, shoot.
                TrcEvent shootCompletionEvent = new TrcEvent(instanceName + ".shootCompletionEvent");
                shootCompletionEvent.setCallback(this::shootCompleted, null);
                shootOp.shoot(shootOpOwner, shootCompletionEvent);
            }
            else
            {
                finish(true);
            }
        }
    }   //onTarget

    /**
     * This method is called when the object has been ejected from the shooter.
     *
     * @param context not used.
     */
    private void shootCompleted(Object context)
    {
        if (shootOffDelay == null)
        {
            tracer.traceInfo(instanceName, "Shoot completed, keeping shooter motor running.");
            finish(true);
        }
        else if (shootOffDelay == 0.0)
        {
            tracer.traceInfo(instanceName, "Shoot completed, stop shooter motor.");
            stopShooter();
            finish(true);
        }
        else
        {
            tracer.traceInfo(
                instanceName, "Shoot completed, delay stopping shooter motor for " + shootOffDelay + "s.");
            // Even if we have a shootOffDelay, don't delay signaling completion.
            if (completionEvent != null)
            {
                completionEvent.signal();
                completionEvent = null;
            }
            shootTimer.set(shootOffDelay, this::timedOut, true);
        }
    }   //shootCompleted

    /**
     * This method is called if the shooter operation has timed out.
     *
     * @param context specifies true for shoot off timeout, false for operation timeout.
     */
    private void timedOut(Object context)
    {
        Boolean completion = (Boolean) context;
        tracer.traceInfo(instanceName, "Timed out: completion=" + completion);
        // Either the operation was timed out or there was a shootOffDelay.
        // Either way, we will turn off the shooter motor.
        stopShooter();
        finish(completion);
    }   //timedOut

    //
    // Shooter motor methods.
    //

    /**
     * This method returns the shooter motor 1 object.
     *
     * @return shooter motor 1.
     */
    public TrcMotor getShooterMotor1()
    {
        return shooterMotor1;
    }   //getShooterMotor1

    /**
     * This method returns the shooter motor 2 object if any.
     *
     * @return shooter motor 2, null if none.
     */
    public TrcMotor getShooterMotor2()
    {
        return shooterMotor2 != null? shooterMotor2: shooterMotor1.getFollower(0);
    }   //getShooterMotor2

    /**
     * This methods returns the shooter motor 1 current power.
     *
     * @return shooter motor 1 current power.
     */
    public double getShooterMotor1Power()
    {
        return shooterMotor1.getPower();
    }   //getShooterMotor1Power

    /**
     * This methods returns the shooter motor 2 current power if any.
     *
     * @return shooter motor 2 current power, null if none.
     */
    public Double getShooterMotor2Power()
    {
        TrcMotor motor2 = getShooterMotor2();
        return motor2 != null? motor2.getPower(): null;
    }   //getShooterMotor2Power

    /**
     * This method returns the shooter motor 1 current velocity.
     *
     * @return shooter motor 1 current velocity in revolutions per second.
     */
    public double getShooterMotor1Velocity()
    {
        return shooterMotor1.getVelocity();
    }   //getShooterMotor1Velocity

    /**
     * This method returns the shooter motor 1 current RPM.
     *
     * @return shooter motor 1 current velocity in RPM.
     */
    public double getShooterMotor1RPM()
    {
        return shooterMotor1.getVelocity() * 60.0;
    }   //getShooterMotor1RPM

    /**
     * This method returns the shooter motor 2 current velocity if any.
     *
     * @return shooter motor 2 current velocity in revolutions per second, null if none.
     */
    public Double getShooterMotor2Velocity()
    {
        TrcMotor motor2 = getShooterMotor2();
        return motor2 != null? motor2.getVelocity(): null;
    }   //getShooterMotor2Velocity

    /**
     * This method returns the shooter motor 2 current RPM if any.
     *
     * @return shooter motor 2 current velocity in RPM, null if none.
     */
    public Double getShooterMotor2RPM()
    {
        TrcMotor motor2 = getShooterMotor2();
        return motor2 != null? motor2.getVelocity() * 60.0: null;
    }   //getShooterMotor2RPM

    /**
     * This method returns the shooter motor 1 current target velocity.
     *
     * @return shooter motor 1 current target velocity in revolutions per second.
     */
    public double getShooterMotor1TargetVelocity()
    {
        return shooterMotor1.getPidTarget();
    }   //getShooterMotor1TargetVelocity

    /**
     * This method returns the shooter motor 1 current target RPM.
     *
     * @return shooter motor 1 current target velocity in RPM.
     */
    public double getShooterMotor1TargetRPM()
    {
        return shooterMotor1.getPidTarget() * 60.0;
    }   //getShooterMotor1TargetRPM

    /**
     * This method returns the shooter motor 2 current target velocity if any.
     *
     * @return shooter motor 2 current target velocity in revolutions per second, null if none.
     */
    public double getShooterMotor2TargetVelocity()
    {
        TrcMotor motor2 = getShooterMotor2();
        return motor2 != null? motor2.getPidTarget(): 0.0;
    }   //getShooterMotor2TargetVelocity

    /**
     * This method returns the shooter motor 2 current target RPM if any.
     *
     * @return shooter motor 2 current target velocity in RPM, null if none.
     */
    public double getShooterMotor2TargetRPM()
    {
        TrcMotor motor2 = getShooterMotor2();
        return motor2 != null? motor2.getPidTarget() * 60.0: 0.0;
    }   //getShooterMotor2TargetRPM

    /**
     * This method sets the shooter motor velocity.
     *
     * @param velocity1 specifies the motor 1 velocity in revolutions per second.
     * @param velocity2 specifies the motor 2 velocity in revolutions per second, ignore if no motor 2.
     */
    public void setShooterMotorVelocity(double velocity1, double velocity2)
    {
        shooterMotor1.setVelocity(null, 0.0, velocity1, 0.0, null);
        if (shooterMotor2 != null)
        {
            shooterMotor2.setVelocity(null, 0.0, velocity2, 0.0, null);
        }
    }   //setShooterMotorVelocity

    /**
     * This method sets the shooter motor velocity in RPM.
     *
     * @param rpm1 specifies the motor 1 velocity in RPM.
     * @param rpm2 specifies the motor 2 velocity in RPM, ignore if no motor 2.
     */
    public void setShooterMotorRPM(double rpm1, double rpm2)
    {
        setShooterMotorVelocity(rpm1/60.0, rpm2/60.0);
    }   //setShooterMotorRPM

    /**
     * This method stops the shooter. Use this method instead of setting shooter velocity to zero because the shooter
     * will coast to a stop instead of stopping abruptly.
     */
    public void stopShooter()
    {
        shooterMotor1.stop();
        if (shooterMotor2 != null)
        {
            shooterMotor2.stop();
        }
    }   //stopShooter

    //
    // Tilt motor methods.
    //

    /**
     * This method returns the tilt motor object if any.
     *
     * @return tilt motor, null if none.
     */
    public TrcMotor getTiltMotor()
    {
        return tiltMotor;
    }   //getTiltMotor

    /**
     * This method returns the current absolute tilt angle from horizontal if any.
     *
     * @return current tilt angle in degrees, null if no tilt motor.
     */
    public Double getTiltAngle()
    {
        return tiltMotor != null? tiltMotor.getPosition(): null;
    }   //getTiltAngle

    /**
     * This method returns the current absolute tilt angle target from horizontal if any.
     *
     * @return current tilt angle target in degrees, null if no tilt motor.
     */
    public Double getTiltAngleTarget()
    {
        return tiltMotor != null? tiltMotor.getPidTarget(): null;
    }   //getTiltAngleTarget

    /**
     * This method sets the tilt angle.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param angle specifies the tilt absolute angle from horizontal in degrees (horizontal is 0-degree).
     * @param completionEvent specifies the event to signal when tilt reached target angle, can be null if not
     *        provided.
     * @param timeout specifies timeout in seconds in case PID control cannot reach target.
     */
    public void setTiltAngle(String owner, double angle, TrcEvent completionEvent, double timeout)
    {
        if (tiltMotor != null)
        {
            tiltMotor.setPosition(owner, 0.0, angle, true, tiltParams.powerLimit, completionEvent, timeout);
        }
    }   //setTiltAngle

    /**
     * This method sets the tilt angle.
     *
     * @param angle specifies the tilt absolute angle from horizontal in degrees (horizontal is 0-degree).
     * @param completionEvent specifies the event to signal when tilt reached target angle, can be null if not
     *        provided.
     * @param timeout specifies timeout in seconds in case PID control cannot reach target.
     */
    public void setTiltAngle(double angle, TrcEvent completionEvent, double timeout)
    {
        setTiltAngle(null, angle, completionEvent, timeout);
    }   //setTiltAngle

    /**
     * This method sets the tilt angle.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param angle specifies the tilt absolute angle from horizontal in degrees (horizontal is 0-degree).
     */
    public void setTiltAngle(String owner, double angle)
    {
        setTiltAngle(owner, angle, null, 0.0);
    }   //setTiltAngle

    /**
     * This method sets the tilt angle.
     *
     * @param angle specifies the tilt absolute angle from horizontal in degrees (horizontal is 0-degree).
     */
    public void setTiltAngle(double angle)
    {
        setTiltAngle(null, angle, null, 0.0);
    }   //setTiltAngle

    /**
     * This method returns the current applied tilt power duty cycle (in the range of -1 to 1) if any.
     *
     * @return current tilt power, null if no tilt motor.
     */
    public Double getTiltPower()
    {
        return tiltMotor != null? tiltMotor.getPower(): null;
    }   //getTiltPower

    /**
     * This method moves tilt up and down with the specified power. It is typically used by TeleOp to control
     * tilting by a joystick value in manual override mode.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param power specifies the power duty cycle used to move tilt (in the range of -1 to 1).
     */
    public void setTiltPower(String owner, double power)
    {
        if (tiltMotor != null)
        {
            tiltMotor.setPower(owner, 0.0, power, 0.0, null);;
        }
    }   //setTiltPower

    /**
     * This method moves tilt up and down with the specified power. It is typically used by TeleOp to control
     * tilting by a joystick value in manual override mode.
     *
     * @param power specifies the power duty cycle used to move tilt (in the range of -1 to 1).
     */
    public void setTiltPower(double power)
    {
        setTiltPower(null, power);
    }   //setTiltPower

    /**
     * This method moves tilt up and down with the specified power using PID control. It is typically used by
     * TeleOp to control tilting by a joystick value.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the upper bound power of the motor.
     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
     */
    public void setTiltPidPower(String owner, double power, boolean holdTarget)
    {
        if (tiltMotor != null)
        {
            tiltMotor.setPidPower(owner, power, tiltParams.minPos, tiltParams.maxPos, holdTarget);
        }
    }   //setTiltPidPower

    /**
     * This method moves tilt up and down with the specified power using PID control. It is typically used by
     * TeleOp to control tilting by a joystick value.
     *
     * @param power specifies the upper bound power of the motor.
     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
     */
    public void setTiltPidPower(double power, boolean holdTarget)
    {
        setTiltPidPower(null, power, holdTarget);
    }   //setTiltPidPower

    /**
     * This method checks if tilt's lower limit switch is active.
     *
     * @return true if active, false otherwise.
     */
    public boolean tiltLowerLimitSwitchActive()
    {
        return tiltMotor != null && tiltMotor.isLowerLimitSwitchActive();
    }   //tiltLowerLimitSwitchActive

    /**
     * This method checks if tilt's upper limit switch is active.
     *
     * @return true if active, false otherwise.
     */
    public boolean tiltUpperLimitSwitchActive()
    {
        return tiltMotor != null && tiltMotor.isUpperLimitSwitchActive();
    }   //tiltUpperLimitSwitchActive

    //
    // Pan motor methods.
    //

    /**
     * This method returns the pan motor object if any.
     *
     * @return pan motor, null if none.
     */
    public TrcMotor getPanMotor()
    {
        return panMotor;
    }   //getPanMotor

    /**
     * This method returns the current absolute pan angle if any.
     *
     * @return current pan angle in degrees, null if no pan motor.
     */
    public Double getPanAngle()
    {
        return panMotor != null? panMotor.getPosition(): null;
    }   //getPanAngle

    /**
     * This method returns the current absolute pan angle target from horizontal if any.
     *
     * @return current pan angle target in degrees, null if no pan motor.
     */
    public Double getPanAngleTarget()
    {
        return panMotor != null? panMotor.getPidTarget(): null;
    }   //getPanAngleTarget

    /**
     * This method sets the pan angle.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param angle specifies the pan absolute angle in degrees.
     * @param completionEvent specifies the event to signal when pan reached target angle, can be null if not
     *        provided.
     * @param timeout specifies timeout in seconds in case PID control cannot reach target.
     */
    public void setPanAngle(String owner, double angle, TrcEvent completionEvent, double timeout)
    {
        if (panMotor != null)
        {
            panMotor.setPosition(owner, 0.0, angle, true, panParams.powerLimit, completionEvent, timeout);
        }
    }   //setPanAngle

    /**
     * This method sets the pan angle.
     *
     * @param angle specifies the pan absolute angle in degrees.
     * @param completionEvent specifies the event to signal when pan reached target angle, can be null if not
     *        provided.
     * @param timeout specifies timeout in seconds in case PID control cannot reach target.
     */
    public void setPanAngle(double angle, TrcEvent completionEvent, double timeout)
    {
        setPanAngle(null, angle, completionEvent, timeout);
    }   //setPanAngle

    /**
     * This method sets the pan angle.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param angle specifies the pan absolute angle in degrees.
     */
    public void setPanAngle(String owner, double angle)
    {
        setPanAngle(owner, angle, null, 0.0);
    }   //setPanAngle

    /**
     * This method sets the pan angle.
     *
     * @param angle specifies the pan absolute angle in degrees.
     */
    public void setPanAngle(double angle)
    {
        setPanAngle(null, angle, null, 0.0);
    }   //setPanAngle

    /**
     * This method returns the current applied pan power duty cycle (in the range of -1 to 1) if any.
     *
     * @return current pan power, null if no pan motor.
     */
    public Double getPanPower()
    {
        return panMotor != null? panMotor.getPower(): null;
    }   //getPanPower

    /**
     * This method moves pan left and right with the specified power. It is typically used by TeleOp to control
     * panning by a joystick value in manual override mode.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param power specifies the power duty cycle used to move pan (in the range of -1 to 1).
     */
    public void setPanPower(String owner, double power)
    {
        if (panMotor != null)
        {
            panMotor.setPower(owner, 0.0, power, 0.0, null);;
        }
    }   //setPanPower

    /**
     * This method moves pan left and right with the specified power. It is typically used by TeleOp to control
     * panning by a joystick value in manual override mode.
     *
     * @param power specifies the power duty cycle used to move pan (in the range of -1 to 1).
     */
    public void setPanPower(double power)
    {
        setPanPower(null, power);
    }   //setPanPower

    /**
     * This method moves pan left and right with the specified power using PID control. It is typically used by
     * TeleOp to control panning by a joystick value.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the upper bound power of the motor.
     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
     */
    public void setPanPidPower(String owner, double power, boolean holdTarget)
    {
        if (panMotor != null)
        {
            panMotor.setPidPower(owner, power, panParams.minPos, panParams.maxPos, holdTarget);
        }
    }   //setPanPidPower

    /**
     * This method moves pan left and right with the specified power using PID control. It is typically used by
     * TeleOp to control panning by a joystick value.
     *
     * @param power specifies the upper bound power of the motor.
     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
     */
    public void setPanPidPower(double power, boolean holdTarget)
    {
        setPanPidPower(null, power, holdTarget);
    }   //setPanPidPower

    /**
     * This method checks if pan's lower limit switch is active.
     *
     * @return true if active, false otherwise.
     */
    public boolean panLowerLimitSwitchActive()
    {
        return panMotor != null && panMotor.isLowerLimitSwitchActive();
    }   //panLowerLimitSwitchActive

    /**
     * This method checks if pan's upper limit switch is active.
     *
     * @return true if active, false otherwise.
     */
    public boolean panUpperLimitSwitchActive()
    {
        return panMotor != null && panMotor.isUpperLimitSwitchActive();
    }   //panUpperLimitSwitchActive

}   //class TrcShooter
