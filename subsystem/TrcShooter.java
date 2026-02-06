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

import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent auto-assist shooter subsystem. It consists of a shooter motor and
 * optionally a tilt motor and/or a pan motor. It provides methods to automate the shooting operation which includes
 * aiming by panning and tilting to the specified angles and spinning the shooter motor to the specified velocity.
 * It then uses the caller provided shoot method to shoot the object and signals completion if necessary.
 */
public class TrcShooter implements TrcExclusiveSubsystem
{
    private static final boolean COMPENSATE_ROBOT_ROTATION = false;

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
        private final double powerLimit;
        private final double minPos, maxPos;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param powerLimit specifies the max power limit of the motor.
         * @param minPos specifies the min position.
         * @param maxPos specifies the max position.
         */
        public PanTiltParams(double powerLimit, double minPos, double maxPos)
        {
            this.powerLimit = powerLimit;
            this.minPos = minPos;
            this.maxPos = maxPos;
        }   //PanTiltParams
    }   //class PanTiltParams

    public static class AimInfo
    {
        public TrcPose2D targetPose;
        public Double flywheel1RPM;
        public Double flywheel2RPM;
        public Double panAngle;
        public Double tiltAngle;
        public Double timeOfFlight;

        public AimInfo(
            TrcPose2D targetPose, Double flywheel1RPM, Double flywheel2RPM, Double panAngle, Double tiltAngle,
            Double timeOfFlight)
        {
            this.targetPose = targetPose;
            this.flywheel1RPM = flywheel1RPM;
            this.flywheel2RPM = flywheel2RPM;
            this.panAngle = panAngle;
            this.tiltAngle = tiltAngle;
            this.timeOfFlight = timeOfFlight;
        }   //AimInfo

        @Override
        public AimInfo clone()
        {
            return new AimInfo(targetPose, flywheel1RPM, flywheel2RPM, panAngle, tiltAngle, timeOfFlight);
        }   //clone

        @Override
        public String toString()
        {
            return "(TargetPose=" + targetPose +
                   ", RPM1=" + flywheel1RPM +
                   ", RPM2=" + flywheel2RPM +
                   ", pan=" + panAngle +
                   ", tilt=" + tiltAngle +
                   ", ToF=" + timeOfFlight + ")";
        }   //toString
    }   //class AimInfo

    public interface AimInfoSource
    {
        AimInfo getAimInfo(TrcPose2D targetPose);
    }   //interface AimInfoSource

    public static class AimConvergenceStats
    {
        public enum ExitReason
        {
            CONVERGED,
            DIVERGING,
            MAX_ITERATIONS,
            NULL_AIM,
            SKIPPED_NO_MOTION
        }

        public ExitReason exitReason = ExitReason.MAX_ITERATIONS;
        public int iterationsUsed = 0;
        public double initialTof = 0.0;
        public double finalTof = 0.0;
        public double finalTofError = 0.0;
        public double finalTofDelta = 0.0;
        public double usedTof = 0.0;
        public TrcPose2D compensation = null;
        public double maxCompensationMag = 0.0;
        public double rawYawComp = 0.0;
        public double yawComp = 0.0;

        @Override
        public String toString()
        {
            return "exitReason=" + exitReason +
                   "\niterations=" + iterationsUsed +
                   "\ninitialTof=" + initialTof +
                   "\nfinalTof=" + finalTof +
                   "\nfinalTofError=" + finalTofError +
                   "\nfinalTofDelta=" + finalTofDelta +
                   "\nusedTof=" + usedTof +
                   "\ncompensation=" + compensation +
                   "\nmaxCompMag=" + maxCompensationMag +
                   "\nyawComp=" + rawYawComp + "/" + yawComp + ")";
        }   //toString
    }   //class AimConvergenceStats

    private static class GoalTrackingParams
    {
        boolean trackingPaused = false;
        AimInfoSource aimInfoSource = null;
    }   //class GoalTrackingParams

    private static class ShooterState
    {
        private boolean active = false;
        private TrcEvent shooterReadyEvent = null;
        private TrcEvent shooter1OnTargetEvent = null;
        private TrcEvent shooter2OnTargetEvent = null;
        private TrcEvent tiltOnTargetEvent = null;
        private TrcEvent panOnTargetEvent = null;
    }   //class ShooterState

    private final ShooterState shooterState = new ShooterState();
    private final GoalTrackingParams goalTrackingParams = new GoalTrackingParams();
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
    private final TrcTaskMgr.TaskObject goalTrackingTaskObj;

    private String currOwner = null;
    private TrcEvent completionEvent = null;
    private ShootOperation shootOp = null;
    private String shootOpOwner = null;
    private Double shootOffDelay = null;
    private Double maxShooter1MaxRPM = null;
    private Double maxShooter2MaxRPM = null;

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
        goalTrackingTaskObj = TrcTaskMgr.createTask(instanceName + ".goalTracking", this::goalTrackingTask);
    }   //TrcShooter

    /**
     * This method checks if the shooter is active.
     *
     * @return true if shooter is active, false otherwise.
     */
    public boolean isActive()
    {
        synchronized (shooterState)
        {
            return shooterState.active;
        }
    }   //isActive

    /**
     * This method is called when the shooter operation is finished or canceled.
     *
     * @param completed specifies true if the operation is completed, false if canceled.
     */
    private void finish(boolean completed)
    {
        synchronized (shooterState)
        {
            aimTimer.cancel();
            shootTimer.cancel();

            if (!completed)
            {
                // The operation was canceled, stop the shooter motor.
                stopShooter();
                if (tiltMotor != null)
                {
                    tiltMotor.cancel();
                }

                if (panMotor != null)
                {
                    panMotor.cancel();
                }
            }
            shootOp = null;
            shootOpOwner = null;
            shootOffDelay = null;

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

            shooterState.active = false;
            if (shooterState.shooterReadyEvent != null)
            {
                tracer.traceInfo(instanceName, "Signal ShooterReady.");
                shooterState.shooterReadyEvent.signal();
                shooterState.shooterReadyEvent = null;
            }
            // Resume Goal Tracking if it was paused.
            resumeGoalTracking();
        }
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
     * This method stops Goal Tracking.
     */
    private void stopGoalTracking()
    {
        tracer.traceInfo(instanceName, "Stop GoalTracking.");
        if (panMotor != null) panMotor.cancel();
        if (tiltMotor != null) tiltMotor.cancel();
        stopShooter();
    }   //stopGoalTracking

    /**
     * This method is called periodically to perform goal tracking.
     *
     * @param taskType specifies the type of task being run. This may be useful for handling multiple task types.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void goalTrackingTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        synchronized (goalTrackingParams)
        {
            if (!goalTrackingParams.trackingPaused)
            {
                AimInfo aimInfo = goalTrackingParams.aimInfoSource.getAimInfo(null);
                if (aimInfo != null)
                {
                    if (aimInfo.flywheel1RPM != null || aimInfo.flywheel2RPM != null)
                    {
                        setShooterMotorRPM(aimInfo.flywheel1RPM, aimInfo.flywheel2RPM);
                    }

                    if (aimInfo.panAngle != null)
                    {
                        setPanAngle(aimInfo.panAngle);
                    }

                    if (aimInfo.tiltAngle != null)
                    {
                        setTiltAngle(aimInfo.tiltAngle);
                    }
                }
            }
        }
    }   //goalTrackingTask

    /**
     * This method checks if Goal Tracking is enabled.
     *
     * @return true if Goal Tracking is enabled, false if disabled.
     */
    public boolean isGoalTrackingEnabled()
    {
        synchronized (goalTrackingParams)
        {
            return goalTrackingParams.aimInfoSource != null;
        }
    }   //isGoalTrackingEnabled

    /**
     * This method checks if Goal Tracking is paused.
     *
     * @return true if goal tracking is enabled but paused, false otherwise.
     */
    public boolean isGoalTrackingPaused()
    {
        synchronized (goalTrackingParams)
        {
            return goalTrackingParams.aimInfoSource != null && goalTrackingParams.trackingPaused;
        }
    }   //isGoalTrackingPaused

    /**
     * This method enables/disables Goal Tracking.
     *
     * @param aimInfoSource specifies the method to call to get Aim Info to enable, null to disable Goal Tracking.
     */
    public void setGoalTrackingEnabled(AimInfoSource aimInfoSource)
    {
        synchronized (goalTrackingParams)
        {
            goalTrackingParams.aimInfoSource = aimInfoSource;
            goalTrackingParams.trackingPaused = false;
            if (aimInfoSource != null)
            {
                tracer.traceInfo(instanceName, "Enable GoalTracking.");
                // Cancel previous operation if any.
                finish(false);
                goalTrackingTaskObj.registerTask(TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
            }
            else
            {
                tracer.traceInfo(instanceName, "Disable Goal Tracking.");
                stopGoalTracking();
                goalTrackingTaskObj.unregisterTask();
            }
        }
    }   //setGoalTrackingEnabled

    /**
     * This method pauses Goal Tracking so that we can perform other operations such as aimShooter that needs the
     * control of the shooter.
     */
    public void pauseGoalTracking()
    {
        synchronized (goalTrackingParams)
        {
            if (goalTrackingParams.aimInfoSource != null)
            {
                tracer.traceInfo(instanceName, "Pause GoalTracking.");
                goalTrackingParams.trackingPaused = true;
            }
        }
    }   //pauseGoalTracking

    /**
     * This method resumes Goal Tracking from a previous pause.
     */
    public void resumeGoalTracking()
    {
        synchronized (goalTrackingParams)
        {
            if (goalTrackingParams.aimInfoSource != null && goalTrackingParams.trackingPaused)
            {
                tracer.traceInfo(instanceName, "Resume GoalTracking.");
                goalTrackingParams.trackingPaused = false;
            }
        }
    }   //resumeGoalTracking

    /**
     * This method enables power mode on the shooter motors. In power mode, it will do open-loop control of the
     * flywheel using percentage power of targetRPM/maxRPM. This is a failsafe mode in case the flywheel encoder
     * has malfunctioned.
     *
     * @param maxShooter1MaxRPM specifies maximum shooter motor RPM for enabling power mode,
     *        null for disabling power mode.
     * @param maxShooter2MaxVel specifies maximum shooter motor RPM, null if motor 2 does not exist.
     */
    public void enableShooterPowerMode(Double maxShooter1MaxRPM, Double maxShooter2MaxRPM)
    {
        this.maxShooter1MaxRPM = maxShooter1MaxRPM;
        this.maxShooter2MaxRPM = maxShooter2MaxRPM;
    }   //enableShooterPowerMode

    /**
     * This method disables power mode on the shooter motors.
     */
    public void disableShooterPowerMode()
    {
        this.maxShooter1MaxRPM = null;
        this.maxShooter2MaxRPM = null;
    }   //disableShooterPowerMode

    /**
     * This method checks if the shooter motor power mode is enabled.
     *
     * @return true if power mode is enabled, false if disabled.
     */
    public boolean isShooterPowerModeEnabled()
    {
        // Only need to check max velocity of motor 1. Max velocity of motor 2 could be null because it doesn't exist,
        // not because power mode is disabled.
        return maxShooter1MaxRPM != null;
    }   //isShooterPowerModeEnabled

    /**
     * This method compensates the AimInfo by the motion of the robot.
     *
     * @param driveBase specifies the drive base object.
     * @param aimInfoSource specifies the method to call to recompute AimInfo by target pose.
     * @param aimInfo specifies the original AimInfo.
     * @param tofErrorThreshold specifies the time-of-flight error threshold to terminate iterations.
     * @param maxIterations specifies the maximum number of iterations.
     * @param stats specifies the ConvergenceStat object to be filled in by stat info, null if not provided.
     * @return compensated Aim Info.
     */
    public AimInfo compensateRobotMotion(
        TrcDriveBase driveBase, AimInfoSource aimInfoSource, AimInfo aimInfo, double tofErrorThreshold,
        int maxIterations, AimConvergenceStats stats)
    {
        AimInfo compensatedAimInfo = aimInfo;
        TrcPose2D fieldVel = driveBase.getFieldVelocity();
        double omega = COMPENSATE_ROBOT_ROTATION? driveBase.getTurnRate(): 0.0;
        AimConvergenceStats.ExitReason exitReason = null;
        int iterations = 0;
        TrcPose2D compensation = new TrcPose2D();   // default zero pose
        double maxCompensationMag = 0.0;
        double tofError = Double.NaN;
        double rawYawComp = 0.0;
        double yawComp = 0.0;
        double usedTof = 0.0;

        // Only compensate if robot is moving linearly or rotating
        if (Math.hypot(fieldVel.x, fieldVel.y) > 0.01 || Math.abs(omega) > 1.0)
        {
            AimInfo currAimInfo = aimInfo;
            double lastTofError = Double.NaN;
            // Rotate field velocity into robot frame (CW-positive convention)
            double headingRad = Math.toRadians(driveBase.getHeading());
            double cos = Math.cos(headingRad);
            double sin = Math.sin(headingRad);
            double vxRobot =  fieldVel.x * cos + fieldVel.y * sin;
            double vyRobot = -fieldVel.x * sin + fieldVel.y * cos;

            maxIterations = Math.min(maxIterations, 5);
            while (iterations < maxIterations)
            {
                usedTof = TrcUtil.clipRange(currAimInfo.timeOfFlight, 0.05, 2.0);
                rawYawComp = omega * usedTof;
                // yawLimit scales inversely with √ToF to avoid over-rotating at long shots.
                double yawLimit = TrcUtil.clipRange(45.0 / Math.sqrt(usedTof), 10.0, 45.0);
                yawComp = TrcUtil.clipRange(rawYawComp, -yawLimit, yawLimit);

                compensation = new TrcPose2D(-vxRobot * usedTof, -vyRobot * usedTof, yawComp);
                double compensationMag = Math.hypot(compensation.x, compensation.y);
                TrcPose2D compensatedPose = currAimInfo.targetPose.addRelativePose(compensation);

                if (compensationMag > maxCompensationMag)
                {
                    maxCompensationMag = compensationMag;
                }

                compensatedAimInfo = aimInfoSource.getAimInfo(compensatedPose);
                if (compensatedAimInfo == null)
                {
                    compensatedAimInfo = currAimInfo;
                    exitReason = AimConvergenceStats.ExitReason.NULL_AIM;
                }
                else
                {
                    tofError = Math.abs(compensatedAimInfo.timeOfFlight - currAimInfo.timeOfFlight);
                    if (tofError <= tofErrorThreshold)
                    {
                        exitReason = AimConvergenceStats.ExitReason.CONVERGED;
                    }
                    else if (!Double.isNaN(lastTofError) && tofError > lastTofError * 1.2)
                    {
                        exitReason = AimConvergenceStats.ExitReason.DIVERGING;
                    }
                    lastTofError = tofError;
                }

                iterations++;
                if (exitReason != null)
                {
                    if (exitReason != AimConvergenceStats.ExitReason.CONVERGED)
                    {
                        iterations--;
                    }
                    break;
                }
                currAimInfo = compensatedAimInfo;
            }

            if (exitReason == null)
            {
                exitReason = AimConvergenceStats.ExitReason.MAX_ITERATIONS;
            }
        }
        else
        {
            exitReason = AimConvergenceStats.ExitReason.SKIPPED_NO_MOTION;
            tofError = 0.0;
            usedTof = 0.0;
        }

        if (stats != null)
        {
            stats.iterationsUsed = iterations;
            stats.initialTof = aimInfo.timeOfFlight;
            stats.finalTof = compensatedAimInfo.timeOfFlight;
            stats.finalTofError =
                exitReason != AimConvergenceStats.ExitReason.SKIPPED_NO_MOTION &&
                exitReason != AimConvergenceStats.ExitReason.NULL_AIM? tofError: 0.0;
            stats.finalTofDelta = compensatedAimInfo.timeOfFlight - aimInfo.timeOfFlight;
            stats.usedTof = usedTof;
            stats.compensation = compensation;
            stats.rawYawComp = rawYawComp;
            stats.yawComp = yawComp;
            stats.maxCompensationMag = maxCompensationMag;
            stats.exitReason = exitReason;

            tracer.traceDebug(instanceName, "ConvergenceState=%s", stats);
        }

        return compensatedAimInfo;
    }   // compensateRobotMotion

    /**
     * This method waits for the shooter finished aiming the target and will signal the given event.
     *
     * @param event specifies the event to signal when aiming is on-target.
     */
    public void waitForShooterReady(TrcEvent event)
    {
        synchronized (shooterState)
        {
            if (!shooterState.active)
            {
                event.signal();
            }
            else
            {
                shooterState.shooterReadyEvent = event;
            }
        }
    }   //waitForShooterReady

    /**
     * This method sets the shooter velocity and the tilt/pan angles if tilt/pan exist. This method is asynchronous.
     * When both shooter velocity and tilt/pan positions have reached target and if shoot method is provided, it will
     * shoot and signal an event if provided.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param aimInfo specifies the AimInfo for aiming the target.
     * @param event specifies an event to signal when both reached target, can be null if not provided.
     * @param timeout specifies maximum timeout period, can be zero if no timeout.
     * @param shootOp specifies the shoot method, can be null if aim only.
     * @param shootOffDelay specifies the delay in seconds to turn off shooter after shooting, or zero if no delay
     *        (turn off immediately), only applicable if shootOp is not null. Can also be null if keeping the shooter
     *        on.
     */
    public void aimShooter(
        String owner, AimInfo aimInfo, TrcEvent event, double timeout, ShootOperation shootOp, Double shootOffDelay)
    {
        tracer.traceDebug(
            instanceName,
            "owner=" + owner +
            ", currOwner=" + getCurrentOwner() +
            ", aimInfo=" + aimInfo +
            ", event=" + event +
            ", timeout=" + timeout +
            ", aimOnly=" + (shootOp == null) +
            ", shootOffDelay=" + shootOffDelay);
        if (isGoalTrackingEnabled())
        {
            pauseGoalTracking();
        }
        // Caller specifies an owner but has not acquired ownership, let's acquire ownership on its behalf.
        if (owner != null && !hasOwnership(owner) && acquireExclusiveAccess(owner))
        {
            currOwner = owner;
        }

        if (validateOwnership(owner))
        {
            synchronized (shooterState)
            {
                this.completionEvent = event;
                this.shootOp = shootOp;
                this.shootOpOwner = shootOp != null ? owner : null;
                this.shootOffDelay = shootOffDelay;

                if (maxShooter1MaxRPM != null)
                {
                    // Shooter is in power mode.
                    shooterState.shooter1OnTargetEvent = null;
                    shooterMotor1.setPower(TrcUtil.clipRange(aimInfo.flywheel1RPM/maxShooter1MaxRPM));
                }
                else
                {
                    // Shooter is in velocity mode.
                    shooterState.shooter1OnTargetEvent = new TrcEvent(instanceName + ".shooter1OnTarget");
                    shooterState.shooter1OnTargetEvent.setCallback(this::onTarget, null);
                    shooterMotor1.setVelocity(
                        0.0, aimInfo.flywheel1RPM/60.0, 0.0, shooterState.shooter1OnTargetEvent);
                }

                if (shooterMotor2 != null)
                {
                    if (maxShooter2MaxRPM != null)
                    {
                        shooterState.shooter2OnTargetEvent = null;
                        shooterMotor2.setPower(TrcUtil.clipRange(aimInfo.flywheel2RPM/maxShooter2MaxRPM));
                    }
                    else
                    {
                        shooterState.shooter2OnTargetEvent = new TrcEvent(instanceName + ".shooter2OnTarget");
                        shooterState.shooter2OnTargetEvent.setCallback(this::onTarget, null);
                        shooterMotor2.setVelocity(
                            0.0, aimInfo.flywheel2RPM/60.0, 0.0, shooterState.shooter2OnTargetEvent);
                    }
                }
                else
                {
                    shooterState.shooter2OnTargetEvent = null;
                }

                if (tiltMotor != null && aimInfo.tiltAngle != null)
                {
                    shooterState.tiltOnTargetEvent = new TrcEvent(instanceName + ".tiltOnTarget");
                    shooterState.tiltOnTargetEvent.setCallback(this::onTarget, null);
                    tiltMotor.setPosition(
                        0.0, aimInfo.tiltAngle, true, tiltParams.powerLimit, shooterState.tiltOnTargetEvent);
                }
                else
                {
                    shooterState.tiltOnTargetEvent = null;
                }

                if (panMotor != null && aimInfo.panAngle != null)
                {
                    shooterState.panOnTargetEvent = new TrcEvent(instanceName + ".panOnTarget");
                    shooterState.panOnTargetEvent.setCallback(this::onTarget, null);
                    panMotor.setPosition(
                        0.0, aimInfo.panAngle, true, panParams.powerLimit, shooterState.panOnTargetEvent);
                }
                else
                {
                    shooterState.panOnTargetEvent = null;
                }

                if (timeout > 0.0)
                {
                    aimTimer.set(timeout, this::timedOut, false);
                }

                shooterState.active = true;
            }
        }
    }   //aimShooter

    /**
     * This method sets the shooter velocity and the tilt/pan angles if tilt/pan exist. This method is asynchronous.
     * When both shooter velocity and tilt/pan positions have reached target and if shoot method is provided, it will
     * shoot and signal an event if provided.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param aimInfo specifies the AimInfo for aiming the target.
     * @param event specifies an event to signal when both reached target, can be null if not provided.
     * @param timeout specifies maximum timeout period, can be zero if no timeout.
     */
    public void aimShooter(String owner, AimInfo aimInfo, TrcEvent event, double timeout)
    {
        aimShooter(owner, aimInfo, event, timeout, null, null);
    }   //aimShooter

    /**
     * This method sets the shooter velocity and the tilt/pan angles if tilt/pan exist. This method is asynchronous.
     * When both shooter velocity and tilt/pan positions have reached target and if shoot method is provided, it will
     * shoot and signal an event if provided.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param aimInfo specifies the AimInfo for aiming the target.
     */
    public void aimShooter(String owner, AimInfo aimInfo)
    {
        aimShooter(owner, aimInfo, null, 0.0, null, null);
    }   //aimShooter

    /**
     * This method sets the shooter velocity and the tilt/pan angles if tilt/pan exist. This method is asynchronous.
     * When both shooter velocity and tilt/pan positions have reached target and if shoot method is provided, it will
     * shoot and signal an event if provided.
     *
     * @param aimInfo specifies the AimInfo for aiming the target.
     */
    public void aimShooter(AimInfo aimInfo)
    {
        aimShooter(null, aimInfo, null, 0.0, null, null);
    }   //aimShooter

    /**
     * This method is called when the shooter has reached target velocity or tilt/pan has reached target positions.
     *
     * @param context not used.
     * @param canceled specifies true if canceled.
     */
    private void onTarget(Object context, boolean canceled)
    {
        synchronized (shooterState)
        {
            tracer.traceDebug(
                instanceName,
                "canceled=" + canceled +
                ",shooter1Event=" + shooterState.shooter1OnTargetEvent +
                ",shooter2Event=" + shooterState.shooter2OnTargetEvent +
                ", tiltEvent=" + shooterState.tiltOnTargetEvent +
                ", panEvent=" + shooterState.panOnTargetEvent +
                ", aimOnly=" + (shootOp == null));
            if (!canceled)
            {
                if ((shooterState.shooter1OnTargetEvent == null || shooterState.shooter1OnTargetEvent.isSignaled()) &&
                    (shooterState.shooter2OnTargetEvent == null || shooterState.shooter2OnTargetEvent.isSignaled()) &&
                    (shooterState.tiltOnTargetEvent == null || shooterState.tiltOnTargetEvent.isSignaled()) &&
                    (shooterState.panOnTargetEvent == null || shooterState.panOnTargetEvent.isSignaled()))
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
            }
        }
    }   //onTarget

    /**
     * This method is called when the object has been ejected from the shooter.
     *
     * @param context not used.
     * @param canceled specifies true if canceled.
     */
    private void shootCompleted(Object context, boolean canceled)
    {
        if (canceled)
        {
            tracer.traceInfo(instanceName, "Shoot canceled.");
            finish(false);
        }
        else if (shootOffDelay == null)
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
            // // Even if we have a shootOffDelay, don't delay signaling completion.
            // if (completionEvent != null)
            // {
            //     completionEvent.signal();
            //     completionEvent = null;
            // }
            shootTimer.set(shootOffDelay, this::timedOut, true);
        }
    }   //shootCompleted

    /**
     * This method is called if the shooter operation has timed out.
     *
     * @param context specifies true for shoot off timeout, false for operation timeout.
     * @param canceled not used.
     */
    private void timedOut(Object context, boolean canceled)
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
     * This methods returns the shooter motor 1 power.
     *
     * @return shooter motor 1 power.
     */
    public double getShooterMotor1Power()
    {
        return shooterMotor1.getPower();
    }   //getShooterMotor1Power

    /**
     * This methods returns the shooter motor 2 power if any.
     *
     * @return shooter motor 2 power, null if none.
     */
    public Double getShooterMotor2Power()
    {
        TrcMotor motor2 = getShooterMotor2();
        return motor2 != null? motor2.getPower(): null;
    }   //getShooterMotor2Power

    /**
     * This methods returns the shooter motor 1 current.
     *
     * @return shooter motor 1 current.
     */
    public double getShooterMotor1Current()
    {
        return shooterMotor1.getCurrent();
    }   //getShooterMotor1Current

    /**
     * This methods returns the shooter motor 2 current if any.
     *
     * @return shooter motor 2 current, null if none.
     */
    public Double getShooterMotor2Current()
    {
        TrcMotor motor2 = getShooterMotor2();
        return motor2 != null? motor2.getCurrent(): null;
    }   //getShooterMotor2Current

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
     * @return shooter motor 2 current target velocity in revolutions per second, 0.0 if none.
     */
    public double getShooterMotor2TargetVelocity()
    {
        TrcMotor motor2 = getShooterMotor2();
        return motor2 != null? motor2.getPidTarget(): 0.0;
    }   //getShooterMotor2TargetVelocity

    /**
     * This method returns the shooter motor 2 current target RPM if any.
     *
     * @return shooter motor 2 current target velocity in RPM, 0.0 if none.
     */
    public double getShooterMotor2TargetRPM()
    {
        TrcMotor motor2 = getShooterMotor2();
        return motor2 != null? motor2.getPidTarget() * 60.0: 0.0;
    }   //getShooterMotor2TargetRPM

    /**
     * This method sets the shooter motor velocity.
     *
     * @param flywheel1RPM specifies the motor 1 RPM.
     * @param flywheel2RPM specifies the motor 2 RPM, ignore if no motor 2.
     */
    public void setShooterMotorRPM(Double flywheel1RPM, Double flywheel2RPM)
    {
        if (!isGoalTrackingEnabled())
        {
            if (flywheel1RPM != null)
            {
                if (maxShooter2MaxRPM != null)
                {
                    shooterMotor1.setPower(TrcUtil.clipRange(flywheel1RPM/maxShooter1MaxRPM));
                }
                else
                {
                    shooterMotor1.setVelocity(null, 0.0, flywheel1RPM/60.0, 0.0, null);
                }
            }

            if (flywheel2RPM != null && shooterMotor2 != null)
            {
                if (maxShooter2MaxRPM != null)
                {
                    shooterMotor2.setPower(TrcUtil.clipRange(flywheel2RPM/maxShooter2MaxRPM));
                }
                else
                {
                    shooterMotor2.setVelocity(null, 0.0, flywheel2RPM/60.0, 0.0, null);
                }
            }
        }
    }   //setShooterMotorRPM

    /**
     * This method sets the shooter motor velocity in RPM.
     *
     * @param velocity1 specifies the motor 1 velocity in rev/sec.
     * @param velocity2 specifies the motor 2 velocity in rev/sec, null if motor2 does not exist.
     */
    public void setShooterMotorVelocity(Double velocity1, Double velocity2)
    {
        setShooterMotorVelocity(velocity1 != null? velocity1*60.0: null, velocity2 != null? velocity2*60.0: null);
    }   //setShooterMotorVelocity

    /**
     * This method stops the shooter. Use this method instead of setting shooter velocity to zero because the shooter
     * will coast to a stop instead of stopping abruptly.
     */
    public void stopShooter()
    {
        shooterMotor1.cancel();
        if (shooterMotor2 != null)
        {
            shooterMotor2.cancel();
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
        if (tiltMotor != null && !isGoalTrackingEnabled())
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
        if (tiltMotor != null && !isGoalTrackingEnabled())
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
        if (tiltMotor != null && !isGoalTrackingEnabled())
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
        if (panMotor != null && !isGoalTrackingEnabled())
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
        if (panMotor != null && !isGoalTrackingEnabled())
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
        if (panMotor != null && !isGoalTrackingEnabled())
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
