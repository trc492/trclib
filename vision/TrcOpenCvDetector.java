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

package trclib.vision;

import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Locale;

import trclib.robotcore.TrcDbgTrace;

/**
 * This class implements a generic OpenCV detector. Typically, it is extended by a specific detector that provides
 * the pipeline to process an image for detecting objects using OpenCV APIs.
 */
public abstract class TrcOpenCvDetector implements TrcVisionProcessor<Mat, TrcOpenCvDetector.DetectedObject<?>>
{
    /**
     * This class encapsulates info of the detected object. It extends TrcVisionTargetInfo.ObjectInfo that requires
     * it to provide a method to return the detected object rect.
     */
    public static abstract class DetectedObject<O> implements TrcVisionTargetInfo.ObjectInfo
    {
        public final String label;
        public final O object;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param label specifies the object label.
         * @param object specifies the contour of the object.
         */
        public DetectedObject(String label, O object)
        {
            this.label = label;
            this.object = object;
        }   //DetectedObject

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "label=%s,boundingRect=%s,area=%.1f,rotatedRect=(%.1f,%.1f,%.1f)",
                label, getObjectRect(), getObjectArea(), getPixelWidth(), getPixelHeight(), getRotatedAngle());
        }   //toString

    }   //class DetectedObject

    /**
     * This interface provides a method for filtering false positive objects in the detected target list.
     */
    public interface FilterTarget
    {
        boolean validateTarget(DetectedObject<?> object);
    }   //interface FilterTarget

    public final TrcDbgTrace tracer;
    protected final String instanceName;
    private final TrcHomographyMapper homographyMapper;
    private final TrcVisionTask<Mat, DetectedObject<?>> visionTask;
    private volatile TrcOpenCvPipeline<DetectedObject<?>> openCvPipeline = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numImageBuffers specifies the number of image buffers to allocate.
     * @param cameraRect specifies the camera rectangle for Homography Mapper, can be null if not provided.
     * @param worldRect specifies the world rectangle for Homography Mapper, can be null if not provided.
     */
    public TrcOpenCvDetector(
        String instanceName, int numImageBuffers, TrcHomographyMapper.Rectangle cameraRect,
        TrcHomographyMapper.Rectangle worldRect)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        if (cameraRect != null && worldRect != null)
        {
            homographyMapper = new TrcHomographyMapper(cameraRect, worldRect);
        }
        else
        {
            homographyMapper = null;
        }
        //
        // Pre-allocate the image buffers.
        //
        Mat[] imageBuffers = new Mat[numImageBuffers];
        for (int i = 0; i < imageBuffers.length; i++)
        {
            imageBuffers[i] = new Mat();
        }

        visionTask = new TrcVisionTask<>(instanceName, this, imageBuffers);
    }   //TrcOpenCvDetector

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
     * This method sets the vision task processing interval.
     *
     * @param interval specifies the processing interval in msec. If 0, process as fast as the CPU can run.
     */
    public void setProcessingInterval(long interval)
    {
        visionTask.setProcessingInterval(interval);
    }   //setProcessingInterval

    /**
     * This method returns the vision task processing interval.
     *
     * @return vision task processing interval in msec.
     */
    public long getProcessingInterval()
    {
        return visionTask.getProcessingInterval();
    }   //getProcessingInterval

    /**
     * This method sets the OpenCV pipeline to be used for the detection and enables it.
     *
     * @param pipeline specifies the pipeline to be used for detection, can be null to disable vision.
     */
    public synchronized void setPipeline(TrcOpenCvPipeline<DetectedObject<?>> pipeline)
    {
        if (pipeline != openCvPipeline)
        {
            // Pipeline has changed.
            if (pipeline != null)
            {
                pipeline.reset();
            }
            openCvPipeline = pipeline;
            visionTask.setTaskEnabled(pipeline != null);
        }
    }   //setPipeline

    /**
     * This method returns the current pipeline.
     *
     * @return current pipeline, null if no set pipeline.
     */
    public TrcOpenCvPipeline<DetectedObject<?>> getPipeline()
    {
        return openCvPipeline;
    }   //getPipeline

    /**
     * This method returns an array of detected targets from Grip vision.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param objGroundOffset specifies the object ground offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return array of detected target info.
     */
    @SuppressWarnings("unchecked")
    public TrcVisionTargetInfo<DetectedObject<?>>[] getDetectedTargetsInfo(
        FilterTarget filter, Comparator<? super TrcVisionTargetInfo<DetectedObject<?>>> comparator,
        double objGroundOffset, double cameraHeight)
    {
        TrcVisionTargetInfo<DetectedObject<?>>[] detectedTargets = null;
        DetectedObject<?>[] objects = visionTask.getDetectedObjects();

        if (objects != null)
        {
            ArrayList<TrcVisionTargetInfo<DetectedObject<?>>> targetList = new ArrayList<>();

            for (DetectedObject<?> obj : objects)
            {
                if (filter == null || filter.validateTarget(obj))
                {
                    TrcVisionTargetInfo<DetectedObject<?>> targetInfo =
                        new TrcVisionTargetInfo<>(obj, homographyMapper, objGroundOffset, cameraHeight);
                    targetList.add(targetInfo);
                }
            }

            if (!targetList.isEmpty())
            {
                detectedTargets = targetList.toArray(new TrcVisionTargetInfo[0]);
                if (comparator != null && detectedTargets.length > 1)
                {
                    Arrays.sort(detectedTargets, comparator);
                }
            }

            if (detectedTargets != null)
            {
                for (int i = 0; i < detectedTargets.length; i++)
                {
                    tracer.traceDebug(instanceName, "[" + i + "] Target=" + detectedTargets[i]);
                }
            }
        }

        return detectedTargets;
    }   //getDetectedTargetsInfo

    //
    // Implements TrcVisionProcessor interface.
    //

    /**
     * This method is called to detect objects in the acquired image frame.
     *
     * @param input specifies the image to be processed.
     * @return detected objects, null if none detected.
     */
    @Override
    public synchronized DetectedObject<?>[] processFrame(Mat input)
    {
        return openCvPipeline.process(input);
    }   //processFrame

    /**
     * This method returns the selected intermediate output Mat.
     *
     * @return selected output mat.
     */
    @Override
    public Mat getSelectedOutput()
    {
        return openCvPipeline.getSelectedOutput();
    }   //getSelectedOutput

}   //class TrcOpenCvDetector
