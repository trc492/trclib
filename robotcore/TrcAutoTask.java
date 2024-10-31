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

package trclib.robotcore;

/**
 * This class implements auto-assist task. It is intended to be extended by a specific auto-assist task that will
 * implement the abstract methods performing the task.
 */
public abstract class TrcAutoTask<T>
{
    /**
     * This method is called to acquire ownership of all subsystems involved in the auto-assist operation. This is
     * typically done before starting an auto-assist operation.
     *
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    protected abstract boolean acquireSubsystemsOwnership();

    /**
     * This method is called to release ownership of all subsystems involved in the auto-assist operation. This is
     * typically done if the auto-assist operation is completed or canceled.
     */
    protected abstract void releaseSubsystemsOwnership();

    /**
     * This method is called to stop all the subsystems.
     */
    protected abstract void stopSubsystems();

    /**
     * This methods is called periodically to run the auto-assist task state.
     *
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    protected abstract void runTaskState(
        Object params, T state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop);

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final String owner;
    private final TrcTaskMgr.TaskType taskType;
    private final TrcTaskMgr.TaskObject autoTaskObj;
    protected final TrcStateMachine<T> sm;

    private Object taskParams;
    private TrcEvent completionEvent;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the task name.
     * @param owner specifies the owner to acquire ownership, can be null if not requiring ownership.
     * @param taskType specifies the auto-assist task type (e.g. TaskType.FAST_POSTPERIODIC_TASK).
     */
    protected TrcAutoTask(String instanceName, String owner, TrcTaskMgr.TaskType taskType)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.owner = owner;
        this.taskType = taskType;
        autoTaskObj = TrcTaskMgr.createTask(instanceName, this::autoTask);
        sm = new TrcStateMachine<>(instanceName);
    }   //TrcAutoTask

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
     * This method cancels an in progress auto task operation if any.
     */
    public void cancel()
    {
        tracer.traceInfo(instanceName, "Canceling auto task operation.");
        stopAutoTask(false);
    }   //cancel

    /**
     * This method is called by the subclass to start the auto task.
     *
     * @param startState specifies the state to start the state machine.
     * @param taskParams specifies the task parameters.
     * @param completionEvent specifies the event to signal when the task is completed, can be null if none provided.
     */
    protected void startAutoTask(T startState, Object taskParams, TrcEvent completionEvent)
    {
        if (owner == null || acquireSubsystemsOwnership())
        {
            this.taskParams = taskParams;
            this.completionEvent = completionEvent;
            sm.start(startState);
            setTaskEnabled(true);
        }
        else
        {
            tracer.traceWarn(instanceName, "Failed to acquire subsystems ownership.");
        }
    }   //startAutoTask

    /**
     * This method is called by the subclass to cancel the auto-assist operation in progress if any.
     */
    protected void stopAutoTask(boolean completed)
    {
        if (isActive())
        {
            setTaskEnabled(false);
            stopSubsystems();

            if (owner != null)
            {
                releaseSubsystemsOwnership();
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
        }
    }   //stopAutoTask

    /**
     * This method checks if the auto task is active.
     *
     * @return true if auto assist task is active, false otherwise.
     */
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method enables/disables the auto-assist task.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        if (enabled && !autoTaskObj.isRegistered())
        {
            autoTaskObj.registerTask(taskType);
        }
        else if (!enabled && autoTaskObj.isRegistered())
        {
            sm.stop();
            autoTaskObj.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void autoTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        T state = sm.checkReadyAndGetState();

        if (state != null)
        {
            tracer.tracePreStateInfo(sm.toString(), state);
            runTaskState(taskParams, state, taskType, runMode, slowPeriodicLoop);
        }
    }   //autoTask

}   //class TrcAutoTask
