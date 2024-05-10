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

package trclib.archive;

import java.util.concurrent.atomic.AtomicReference;

/**
 * This class implements a platform independent vision task. When enabled, it grabs a frame from the video source,
 * calls the provided vision processor to process the frame and overlays rectangles on the detected objects in the
 * image. This class is to be extended by a platform dependent vision processor.
 *
 * @param <I> specifies the type of the input image.
 * @param <O> specifies the type of the detected objects.
 */
public class TrcVisionTask<I, O>
{
    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcVisionProcessor<I, O> visionProcessor;
    private final I[] imageBuffers;
    private final TrcTaskMgr.TaskObject visionTaskObj;
    private final AtomicReference<O[]> detectedObjects = new AtomicReference<>();
    private volatile boolean taskEnabled = false;
    private int imageIndex = 0;

    private double totalTime = 0.0;
    private long totalFrames = 0;
    private double taskStartTime = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param visionProcessor specifies the vision processor object.
     * @param imageBuffers specifies an array of image buffers.
     */
    public TrcVisionTask(
        String instanceName, TrcVisionProcessor<I, O> visionProcessor, I[] imageBuffers)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.visionProcessor = visionProcessor;
        this.imageBuffers = imageBuffers;
        visionTaskObj = TrcTaskMgr.createTask(instanceName, this::visionTask);
    }   //TrcVisionTask

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
     */
    public void setTraceLevel(TrcDbgTrace.MsgLevel msgLevel)
    {
        tracer.setTraceLevel(msgLevel);
    }   //setTraceLevel

    /**
     * This method enables/disables the vision task. As long as the task is enabled, it will continue to
     * acquire/process images.
     *
     * @param enabled specifies true to enable vision task, false to disable.
     */
    public synchronized void setTaskEnabled(boolean enabled)
    {
        if (enabled && !taskEnabled)
        {
            totalTime = 0.0;
            totalFrames = 0;
            taskStartTime = TrcTimer.getCurrentTime();
            visionTaskObj.registerTask(TrcTaskMgr.TaskType.STANDALONE_TASK);
        }
        else if (!enabled && taskEnabled)
        {
            visionTaskObj.unregisterTask();
        }
        detectedObjects.set(null);
        taskEnabled = enabled;
    }   //setTaskEnabled

    /**
     * This method returns the state of the vision task.
     *
     * @return true if the vision task is enabled, false otherwise.
     */
    public boolean isTaskEnabled()
    {
        return taskEnabled;
    }   //isTaskEnabled

    /**
     * This method sets the vision task processing interval.
     *
     * @param interval specifies the processing interval in msec. If 0, process as fast as the CPU can run.
     */
    public synchronized void setProcessingInterval(long interval)
    {
        visionTaskObj.setTaskInterval(interval);
    }   //setProcessingInterval

    /**
     * This method returns the vision task processing interval.
     *
     * @return vision task processing interval in msec.
     */
    public synchronized long getProcessingInterval()
    {
        return visionTaskObj.getTaskInterval();
    }   //getProcessingInterval

    /**
     * This method returns the last detected objects. Note that this call consumes the objects, meaning if this method
     * is called again before the next frame is finished processing, it will return a null.
     *
     * @return the last detected objects.
     */
    public O[] getDetectedObjects()
    {
        return detectedObjects.getAndSet(null);
    }   //getDetectedObjects

    /**
     * This method runs periodically to do vision processing.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the current robot run mode.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void visionTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        if (visionProcessor.getFrame(imageBuffers[imageIndex]))
        {
            double startTime = TrcTimer.getCurrentTime();
            //
            // Capture an image and subject it for object detection. The object detector produces an array of
            // rectangles representing objects detected.
            //
            O[] objects = visionProcessor.processFrame(imageBuffers[imageIndex]);

            double elapsedTime = TrcTimer.getCurrentTime() - startTime;
            totalTime += elapsedTime;
            totalFrames++;
            tracer.traceDebug(
                instanceName, "AvgProcessTime=%.6f, FrameRate=%f",
                totalTime/totalFrames, totalFrames/(TrcTimer.getCurrentTime() - taskStartTime));

            I output = visionProcessor.getSelectedOutput();
            if (output != null)
            {
                visionProcessor.putFrame(output);
            }

            detectedObjects.set(objects);
            //
            // Switch to the next buffer so that we won't clobber the info while the client is accessing it.
            //
            imageIndex = (imageIndex + 1) % imageBuffers.length;
        }
    }   //visionTask

}   //class TrcVisionTask
