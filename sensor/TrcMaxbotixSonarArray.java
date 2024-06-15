/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib.sensor;

import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.robotcore.TrcTaskMgr;
import trclib.inputoutput.TrcDigitalOutput;
import trclib.timer.TrcTimer;

/**
 * This class implements a platform independent Maxbotix ultrasonic sensor array. The ultrasonic sensors in the array
 * are connected in analog chain mode where only one sensor will ping at a time to eliminate cross talk. This class
 * supports both regular chain config and loop chain config.
 * <a href="https://www.maxbotix.com/documents/LV-MaxSonar-EZ_Datasheet.pdf">...</a>
 */
public class TrcMaxbotixSonarArray
{
    private static final double RANGING_START_PULSE_WIDTH = 0.02;   //in seconds
    private static final double RANGING_PERIOD = 0.05;              //in seconds

    enum State
    {
        PULL_RX_HIGH,
        PULL_RX_LOW,
        DONE
    }   //enum State

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcAnalogInput[] sensors;
    private final TrcDigitalOutput rx;
    private final boolean loopConfig;
    private final TrcTaskMgr.TaskObject rangingTaskObj;
    private final TrcStateMachine<State> sm;
    private final TrcTimer timer;
    private final TrcEvent event;
    private boolean autoRepeat = false;
    private boolean rangingStarted = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensors specifies an array of Maxbotix ultrasonic sensors.
     * @param rx specifies the digital output channel the RX pin is connected to.
     * @param loopConfig specifies true if the sensor array is wired in loop configuration, false otherwise.
     */
    public TrcMaxbotixSonarArray(String instanceName, TrcAnalogInput[] sensors, TrcDigitalOutput rx, boolean loopConfig)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.sensors = sensors;
        this.rx = rx;
        this.loopConfig = loopConfig;
        rangingTaskObj = TrcTaskMgr.createTask(instanceName + ".rangingTask", this::rangingTask);
        sm = new TrcStateMachine<>(instanceName);
        timer = new TrcTimer(instanceName);
        event = new TrcEvent(instanceName);
    }   //TrcMaxbotixSonarArray

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensors specifies an array of Maxbotix ultrasonic sensors.
     * @param rx specifies the digital output channel the RX pin is connected to.
     */
    public TrcMaxbotixSonarArray(String instanceName, TrcAnalogInput[] sensors, TrcDigitalOutput rx)
    {
        this(instanceName, sensors, rx, false);
    }   //TrcMaxbotixSonarArray

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies a single Maxbotix ultrasonic sensor.
     * @param rx specifies the digital output channel the RX pin is connected to.
     */
    public TrcMaxbotixSonarArray(String instanceName, TrcAnalogInput sensor, TrcDigitalOutput rx)
    {
        this(instanceName, new TrcAnalogInput[] {sensor}, rx, false);
    }   //TrcMaxbotixSonarArray

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
     * This method checks if the sonar array is ranging.
     *
     * @return true if the sonar array is ranging, false otherwise.
     */
    public synchronized boolean isRanging()
    {
        return rangingStarted;
    }   //isRanging

    /**
     * This method is called to start the ranging cycle.
     *
     * @param autoRepeat specifies true to auto repeat the ranging cycle, false otherwise. autoRepeat is ignored
     *        if the array is wired in loop config because it is already in continuous ranging mode.
     */
    public synchronized void startRanging(boolean autoRepeat)
    {
        if (!loopConfig)
        {
            this.autoRepeat = autoRepeat;
        }
        setTaskEnabled(true);
        rangingStarted = true;
    }   //startRanging

    /**
     * This method is called to start the ranging cycle.
     */
    public void startRanging()
    {
        startRanging(false);
    }   //startRanging

    /**
     * This method is called to stop the ranging cycle. Ranging cycle can only be stopped if it is not wired in
     * loop config and was set to autoRepeat mode. If the sensor array is wired in loop config, the only way to
     * stop ranging is to remove power.
     */
    public synchronized void stopRanging()
    {
        //
        // In loop config mode, ranging will never stop once started.
        //
        if (!loopConfig)
        {
            if (autoRepeat)
            {
                autoRepeat = false;
                setTaskEnabled(false);
            }
            rangingStarted = false;
        }
    }   //stopRanging

    /**
     * This method returns the distance data of the specifies sensor index.
     *
     * @param sensorIndex specifies the index of the ultrasonic sensor to read its distance data.
     * @return distance data of the specified sensor.
     */
    public TrcSensor.SensorData<Double> getDistance(int sensorIndex)
    {
        return sensors[sensorIndex].getData(0);
    }   //getDistance

    /**
     * This method is called to start the task that generates the RX pulse for ranging to start.
     *
     * @param enabled specifies true to start the task, false otherwise.
     */
    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            rangingTaskObj.registerTask(TrcTaskMgr.TaskType.INPUT_TASK);
            sm.start(State.PULL_RX_HIGH);
        }
        else
        {
            sm.stop();
            rangingTaskObj.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This method is called periodically to run the state machine that generates the RX pulse for ranging.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private synchronized void rangingTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        if (sm.isReady())
        {
            State state = sm.getState();

            tracer.traceDebug(instanceName, "State: " + state);
            switch (state)
            {
                case PULL_RX_HIGH:
                    //
                    // Set RX high for PULSE_WIDTH time.
                    //
                    rx.setState(true);
                    timer.set(RANGING_START_PULSE_WIDTH, event);
                    sm.waitForSingleEvent(event, State.PULL_RX_LOW);
                    break;

                case PULL_RX_LOW:
                    rx.setState(false);
                    timer.set(RANGING_PERIOD*sensors.length, event);
                    sm.waitForSingleEvent(event, !loopConfig && autoRepeat? State.PULL_RX_HIGH: State.DONE);
                    break;

                case DONE:
                    setTaskEnabled(false);
                    break;
            }
        }
    }   //rangingTask

}   //class TrcMaxbotixSonarArray
