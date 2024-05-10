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

package trclib.archive;

/**
 * This class implements Performance Metrics for Vision. It keeps track of the average time for vision to process a
 * frame as well as the process frame rate.
 */
public class TrcVisionPerformanceMetrics
{
    private final String instanceName;
    private double sessionStartTime;
    private double totalProcessedTime;
    private long totalProcessedFrames;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcVisionPerformanceMetrics(String instanceName)
    {
        this.instanceName = instanceName;
        reset();
    }   //TrcVisionPerformanceMetrics

    /**
     * This method resets the pipeline performance metrics. It is typically called before enabling the pipeline.
     */
    public void reset()
    {
        sessionStartTime = TrcTimer.getCurrentTime();
        totalProcessedTime = 0.0;
        totalProcessedFrames = 0;
    }   //reset

    /**
     * This method is called to log the processing time of the pipeline.
     *
     * @param startTime specifies the timestamp when the processing starts.
     */
    public void logProcessingTime(double startTime)
    {
        totalProcessedTime += TrcTimer.getCurrentTime() - startTime;
        totalProcessedFrames++;
    }   //logProcessingTime

    /**
     * This method prints the pipeline performance metrics using the given tracer.
     *
     * @param tracer specifies the tracer to be used to print performance info.
     */
    public void printMetrics(TrcDbgTrace tracer)
    {
        tracer.traceInfo(
            instanceName, "AvgProcessTime=%.6f, FrameRate=%f",
            totalProcessedTime/totalProcessedFrames, totalProcessedFrames/(TrcTimer.getCurrentTime() - sessionStartTime));
    }   //printMetrics

}   //class TrcVisionPerformanceMetrics
