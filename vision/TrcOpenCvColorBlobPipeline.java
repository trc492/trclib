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

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcDbgTrace;
import trclib.timer.TrcTimer;

/**
 * This class implements a generic OpenCV color blob detection pipeline.
 */
public class TrcOpenCvColorBlobPipeline implements TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<?>>
{
    /**
     * This class encapsulates color thresholding operation properties of the pipeline.
     */
    public static class ColorThresholds
    {
        public String name;
        public double[] lowThresholds;
        public double[] highThresholds;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param name specifies the name of the color ranges. This will be used to label the detected object.
         * @param lowThresholds specifies the low threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
         *        {Y, Cr, Cb} etc.)
         * @param highThresholds specifies the high threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
         *        {Y, Cr, Cb} etc.)
         */
        public ColorThresholds(String name, double[] lowThresholds, double[] highThresholds)
        {
            this.name = name;
            this.lowThresholds = lowThresholds;
            this.highThresholds = highThresholds;
        }   //ColorThresholds

        @Override
        public String toString()
        {
            return "(name=" + name +
                   ",lowThreshold=" + Arrays.toString(lowThresholds) +
                   ",highThresholds=" + Arrays.toString(highThresholds) + ")";
        }   //toString

    }   //class ColorThresholds

    /**
     * This class encapsulates all the filter contour parameters.
     */
    public static class FilterContourParams
    {
        public double minArea = 0.0;
        public double minPerimeter = 0.0;
        public double[] widthRange = {0.0, 1000.0};
        public double[] heightRange = {0.0, 1000.0};
        public double[] solidityRange = {0.0, 100.0};
        public double[] verticesRange = {0.0, 1000000.0};
        public double[] aspectRatioRange = {0.0, 1000.0};

        public FilterContourParams setMinArea(double minArea)
        {
            this.minArea = minArea;
            return this;
        }   //setMinArea

        public FilterContourParams setMinPerimeter(double minPerimeter)
        {
            this.minPerimeter = minPerimeter;
            return this;
        }   //setMinPerimeter

        public FilterContourParams setWidthRange(double min, double max)
        {
            this.widthRange[0] = min;
            this.widthRange[1] = max;
            return this;
        }   //setWidthRange

        public FilterContourParams setHeightRange(double min, double max)
        {
            this.heightRange[0] = min;
            this.heightRange[1] = max;
            return this;
        }   //setHeightRange

        public FilterContourParams setSolidityRange(double min, double max)
        {
            this.solidityRange[0] = min;
            this.solidityRange[1] = max;
            return this;
        }   //setSolidityRange

        public FilterContourParams setVerticesRange(double min, double max)
        {
            this.verticesRange[0] = min;
            this.verticesRange[1] = max;
            return this;
        }   //setVerticesRange

        public FilterContourParams setAspectRatioRange(double min, double max)
        {
            this.aspectRatioRange[0] = min;
            this.aspectRatioRange[1] = max;
            return this;
        }   //setAspectRatioRange

        public void setAs(FilterContourParams other)
        {
            minArea = other.minArea;
            minPerimeter = other.minPerimeter;
            System.arraycopy(other.widthRange, 0, this.widthRange, 0, this.widthRange.length);
            System.arraycopy(other.heightRange, 0, this.heightRange, 0, this.heightRange.length);
            System.arraycopy(other.solidityRange, 0, this.solidityRange, 0, this.solidityRange.length);
            System.arraycopy(other.verticesRange, 0, this.verticesRange, 0, this.verticesRange.length);
            System.arraycopy(other.aspectRatioRange, 0, this.aspectRatioRange, 0, this.aspectRatioRange.length);
        }   //setAs

        @Override
        public FilterContourParams clone()
        {
            return new FilterContourParams()
                .setMinArea(minArea)
                .setMinPerimeter(minPerimeter)
                .setWidthRange(widthRange[0], widthRange[1])
                .setHeightRange(heightRange[0], heightRange[1])
                .setSolidityRange(solidityRange[0], solidityRange[1])
                .setVerticesRange(verticesRange[0], verticesRange[1])
                .setAspectRatioRange(aspectRatioRange[0], aspectRatioRange[1]);
        }   //clone

        @Override
        public String toString()
        {
            return "minArea=" + minArea +
                   ",minPerim=" + minPerimeter +
                   ",width=(" + widthRange[0] + "," + widthRange[1] + ")" +
                   ",height=(" + heightRange[0] + "," + heightRange[1] + ")" +
                   ",solidity=(" + solidityRange[0] + "," + solidityRange[1] + ")" +
                   ",vertices=(" + verticesRange[0] + "," + verticesRange[1] + ")" +
                   ",aspectRatio=(" + aspectRatioRange[0] + "," + aspectRatioRange[1] + ")";
        }   //toString

    }   //class FilterContourParams

    /**
     * This class contains all the pipeline parameters.
     */
    public static class PipelineParams
    {
        public double objWidth = 0.0;
        public double objHeight = 0.0;
        public Integer colorConversion = null;
        public ArrayList<ColorThresholds> colorThresholdsList;
        public int contourRetrievalMode = Imgproc.RETR_EXTERNAL;
        public FilterContourParams filterContourParams = null;
        public Mat cameraMatrix = null;
        public MatOfDouble distCoeffs = null;
        public TrcPose3D cameraPose = null;
        public MatOfPoint3f objPoints = null;

        /**
         * This method sets the detected object's real world size.
         *
         * @param objWidth specifies the detected object's real world width.
         * @param objHeight specifies the detected object's real world height.
         * @return this object for chaining.
         */
        public PipelineParams setObjectSize(double objWidth, double objHeight)
        {
            this.objWidth = objWidth;
            this.objHeight = objHeight;
            return this;
        }   //setObjectSize

        /**
         * This method sets the parameters for color conversion and color thresholding.
         *
         * @param colorConversion specifies the conversion target color space, can be null if no conversion needed.
         * @param name specifies the name of the color ranges. This will be used to label the detected object.
         * @param lowThresholds specifies the low threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
         *        {Y, Cr, Cb} etc.)
         * @param highThresholds specifies the high threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
         *        {Y, Cr, Cb} etc.)
         * @return this object for chaining.
         */
        public PipelineParams setColorThresholds(
            Integer colorConversion, String name, double[] lowThresholds, double[] highThresholds)
        {
            this.colorConversion = colorConversion;
            this.colorThresholdsList = new ArrayList<>();
            this.colorThresholdsList.add(new ColorThresholds(name, lowThresholds, highThresholds));
            return this;
        }   //setColorThresholds

        /**
         * This method adds another set of color thresholds. This allows the same pipeline to detect different color
         * blobs.
         *
         * @param name specifies the name of the color ranges. This will be used to label the detected object.
         * @param lowThresholds specifies the low threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
         *        {Y, Cr, Cb} etc.)
         * @param highThresholds specifies the high threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
         *        {Y, Cr, Cb} etc.)
         * @return this object for chaining.
         */
        public PipelineParams addColorThresholds(String name, double[] lowThresholds, double[] highThresholds)
        {
            if (colorThresholdsList == null)
            {
                throw new IllegalStateException("Must call setColorThresholds first.");
            }

            colorThresholdsList.add(new ColorThresholds(name, lowThresholds, highThresholds));
            return this;
        }   //addColorThresholds

        /**
         * This method sets contour detection parameters.
         *
         * @param externalContourOnly specifies true to detect external contour only (i.e. no internal holes).
         * @param filterParams specifies the contour filtering parameters.
         * @return this object for chaining.
         */
        public PipelineParams setContourDetectionParams(boolean externalContourOnly, FilterContourParams filterParams)
        {
            this.contourRetrievalMode = externalContourOnly? Imgproc.RETR_EXTERNAL: Imgproc.RETR_LIST;
            this.filterContourParams = filterParams;
            return this;
        }   //setContourDetectionParams

        /**
         * This method sets the parameters used for SolvePnp operation.
         *
         * @param fx specifies the focal length in x.
         * @param fy specifies the focal length in y.
         * @param cx specifies the principal point in x.
         * @param cy specifies the principal point in y.
         * @param distCoeffs specifies an array containing the lens distortion coefficients.
         * @param cameraPose specifies the camera pose from robot center.
         * @return this object for chaining.
         */
        public PipelineParams setSolvePnpParams(
            double fx, double fy, double cx, double cy, double[] distCoeffs, TrcPose3D cameraPose)
        {
            cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
            cameraMatrix.put(
                0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);
            this.distCoeffs = new MatOfDouble(distCoeffs);
            this.cameraPose = cameraPose;
            this.objPoints = new MatOfPoint3f(
                new Point3(-objWidth/2.0, -objHeight/2.0, 0.0),
                new Point3(objWidth/2.0, -objHeight/2.0, 0.0),
                new Point3(objWidth/2.0, objHeight/2.0, 0.0),
                new Point3(-objWidth/2.0, objHeight/2.0, 0.0));
            return this;
        }   //setSolvePnpParams

        @Override
        public String toString()
        {
            return "objWidth=" + objWidth +
                   ",objHeight=" + objHeight +
                   ",colorConversion=" + colorConversion +
                   ",externalContourOnly=" + (contourRetrievalMode == Imgproc.RETR_EXTERNAL) +
                   ",filterContourParams=" + filterContourParams +
                   ",cameraPose=" + cameraPose;
        }   //toString

    }   //class PipelineParams

    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public class DetectedObject extends TrcOpenCvDetector.DetectedObject<MatOfPoint>
    {
        public final RotatedRect rotatedRect;
        public final double rotatedRectAngle;
        public final Point[] vertices = new Point[4];
        public final TrcPose2D objPose;
        public final double pixelWidth, pixelHeight;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param label specifies the object label.
         * @param contour specifies the contour of the detected object.
         */
        public DetectedObject(String label, MatOfPoint contour)
        {
            super(label, contour);
            rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            // The angle OpenCV gives us can be ambiguous, so look at the shape of the rectangle to fix that.
            if (rotatedRect.size.width < rotatedRect.size.height)
            {
                // pixelWidth is the longer edge, swap them.
                rotatedRectAngle = rotatedRect.angle + 90.0;
                pixelWidth = rotatedRect.size.height;
                pixelHeight = rotatedRect.size.width;
            }
            else
            {
                rotatedRectAngle = rotatedRect.angle;
                pixelWidth = rotatedRect.size.width;
                pixelHeight = rotatedRect.size.height;
            }
            // Get the 2D image points from the detected rectangle corners
            rotatedRect.points(vertices);

            // Solve PnP: assuming the object is a rectangle with known dimensions.
            if (pipelineParams.cameraMatrix != null && pipelineParams.distCoeffs != null &&
                Calib3d.solvePnP(
                    // Define the 3D coordinates of the object corners in the object coordinate space
                    pipelineParams.objPoints,                                  // Object points in 3D
                    new MatOfPoint2f(orderPoints(vertices)),    // Corresponding image points
                    pipelineParams.cameraMatrix,
                    pipelineParams.distCoeffs,
                    rvec,
                    tvec))
            {
                objPose = projectPose(rvec, tvec, pipelineParams.cameraPose);
//                objPose = new TrcPose2D(tvec.get(0, 0)[0], tvec.get(2, 0)[0], -(Math.toDegrees(rvec.get(1, 0)[0])));
            }
            else
            {
                // Caller did not provide camera matrix nor distortion coefficients so we can't call solvePnP.
                // return null so caller will determine object pose by Homography.
                objPose = null;
            }
        }   //DetectedObject

        /**
         * This method projects the result from SolvePnP to a 2D pose on the ground.
         *
         * @param rvec specifies the rotational vector from SolvePnP.
         * @param tvec specifies the translational vector from SolvePnP.
         * @param cameraPose specifies the camera's 3D position on the robot.
         * @return projected 2D pose on the ground.
         */
        private TrcPose2D projectPose(Mat rvec, Mat tvec, TrcPose3D cameraPose)
        {
            Mat camera_rvec = cameraRVec(cameraPose);
            Mat camera_tvec = cameraTVec(cameraPose);

            Mat modelToRobot_rvec = new Mat();
            Mat modelToRobot_tvec = new Mat();
            Calib3d.composeRT(rvec, tvec, camera_rvec, camera_tvec, modelToRobot_rvec, modelToRobot_tvec);
            Mat rotMat = new Mat();
            Calib3d.Rodrigues(modelToRobot_rvec, rotMat);

            double theta = Math.atan2(
                -rotMat.get(1, 2)[0],
                -rotMat.get(0, 2)[0]);

            return new TrcPose2D(
                -modelToRobot_tvec.get(1, 0)[0],
                modelToRobot_tvec.get(0, 0)[0],
                -Math.toDegrees(theta)
            );
        }   //projectPose

        /**
         * This method creates a rotational vector from the camera pose.
         *
         * @param cameraPose specifies the camera's 3D position on the robot.
         * @return rotational vector of the camera on the robot.
         */
        private Mat cameraRVec(TrcPose3D cameraPose)
        {
//            trc conventions = yaw is CW around up, pitch is CCW around right, roll is CCW around forward
//            standard conventions = yaw is CCW around up, pitch is CCW around left, roll is CCW around forward
            double yaw = -Math.toRadians(cameraPose.yaw);
            double pitch = -Math.toRadians(cameraPose.pitch);
            double roll = Math.toRadians(cameraPose.roll);

            Rotation rot = new Rotation(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR, yaw, pitch, roll);
            Vector3D axis = rot.getAxis(RotationConvention.VECTOR_OPERATOR);
            double angle = rot.getAngle();

            Mat rvec = new Mat(3, 1, CvType.CV_64F);
            rvec.put(0, 0, axis.getX() * angle);
            rvec.put(1, 0, axis.getY() * angle);
            rvec.put(2, 0, axis.getZ() * angle);
            return rvec;
        }   //cameraRVec

        /**
         * This method creates a translational vector from the camera pose.
         *
         * @param cameraPose specifies the camera's 3D position on the robot.
         * @return translational vector of the camera on the robot.
         */
        private Mat cameraTVec(TrcPose3D cameraPose)
        {
            Mat tvec = new Mat(3, 1, CvType.CV_64F);
            tvec.put(0, 0, cameraPose.x);
            tvec.put(1, 0, cameraPose.y);
            tvec.put(2, 0, cameraPose.z);
            return tvec;
        }   //cameraTVec

        /**
         * This method orders the array of 4 points in the order: top-left, top-right, bottom-right, bottom-left.
         *
         * @param pts specifies the points before ordering.
         * @return ordered points.
         */
        private Point[] orderPoints(Point[] pts)
        {
            // Orders the array of 4 points in the order: top-left, top-right, bottom-right, bottom-left
            Point[] orderedPts = new Point[4];

            // Sum and difference of x and y coordinates
            double[] sum = new double[4];
            double[] diff = new double[4];

            for (int i = 0; i < 4; i++)
            {
                sum[i] = pts[i].x + pts[i].y;
                diff[i] = pts[i].y - pts[i].x;
            }

            // Top-left point has the smallest sum
            int tlIndex = indexOfMin(sum);
            orderedPts[0] = pts[tlIndex];

            // Bottom-right point has the largest sum
            int brIndex = indexOfMax(sum);
            orderedPts[2] = pts[brIndex];

            // Top-right point has the smallest difference
            int trIndex = indexOfMin(diff);
            orderedPts[1] = pts[trIndex];

            // Bottom-left point has the largest difference
            int blIndex = indexOfMax(diff);
            orderedPts[3] = pts[blIndex];

            return orderedPts;
        }   //orderPoints

        /**
         * This method finds the min value in the given array and return its index.
         *
         * @param array specifies the array to search for min value.
         * @return index of min value found.
         */
        private int indexOfMin(double[] array)
        {
            int index = 0;
            double min = array[0];

            for (int i = 1; i < array.length; i++)
            {
                if (array[i] < min)
                {
                    min = array[i];
                    index = i;
                }
            }
            return index;
        }   //indexOfMin

        /**
         * This method finds the max value in the given array and return its index.
         *
         * @param array specifies the array to search for max value.
         * @return index of max value found.
         */
        private int indexOfMax(double[] array)
        {
            int index = 0;
            double max = array[0];

            for (int i = 1; i < array.length; i++)
            {
                if (array[i] > max)
                {
                    max = array[i];
                    index = i;
                }
            }
            return index;
        }   //indexOfMax

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getObjectRect()
        {
            // Get detected object bounding box.
            return Imgproc.boundingRect(object);
        }   //getObjectRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getObjectArea()
        {
            // OpenCv returns the actual area of the object, not just the bounding box.
            return Imgproc.contourArea(object);
        }   //getObjectArea

        /**
         * This method returns the object's pixel width.
         *
         * @return object pixel width, null if not supported.
         */
        @Override
        public Double getPixelWidth()
        {
            return pixelWidth;
        }   //getPixelWidth

        /**
         * This method returns the object's pixel height.
         *
         * @return object pixel height, null if not supported.
         */
        @Override
        public Double getPixelHeight()
        {
            return pixelHeight;
        }   //getPixelHeight

        /**
         * This method returns the object's rotated rectangle angle.
         *
         * @return rotated rectangle angle.
         */
        @Override
        public Double getRotatedRectAngle()
        {
            return rotatedRectAngle;
        }   //getRotatedRectAngle

        /**
         * This method returns the pose of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        @Override
        public TrcPose2D getObjectPose()
        {
            return objPose;
        }   //getObjectPose

        /**
         * This method returns the real world width of the detected object.
         *
         * @return real world width of the detected object.
         */
        @Override
        public Double getObjectWidth()
        {
            return pipelineParams.objWidth;
        }   //getObjectWidth

        /**
         * This method returns the real world depth of the detected object.
         *
         * @return real world depth of the detected object.
         */
        @Override
        public Double getObjectDepth()
        {
            return objPose != null? TrcUtil.magnitude(objPose.x, objPose.y): null;
        }   //getObjectDepth

        /**
         * This method returns the rotated rect vertices of the detected object.
         *
         * @return rotated rect vertices.
         */
        @Override
        public Point[] getRotatedRectVertices()
        {
            return vertices;
        }   //getRotatedRectVertices

    }   //class DetectedObject

    private static final Scalar ANNOTATE_RECT_COLOR = new Scalar(0, 255, 0, 255);
    private static final Scalar ANNOTATE_RECT_WHITE = new Scalar(255, 255, 255, 255);
    private static final int ANNOTATE_RECT_THICKNESS = 1;
    private static final Scalar ANNOTATE_TEXT_COLOR = new Scalar(0, 255, 255, 255);
    private static final double ANNOTATE_FONT_SCALE = 0.3;
    private static final int NUM_INTERMEDIATE_MATS = 7;
    private static final int DEF_BLUR_KERNEL_SIZE = 5;

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final PipelineParams pipelineParams;
    private final Mat[] intermediateMats;
    private final Mat hierarchy = new Mat();
    private final Mat rvec = new Mat();
    private final Mat tvec = new Mat();

    private final AtomicReference<DetectedObject[]> detectedObjectsUpdate = new AtomicReference<>();
    private int intermediateStep = 0;
    private boolean annotateEnabled = false;
    private boolean drawCrosshair = false;
    private boolean drawRotatedRect = false;
    private int morphOp = Imgproc.MORPH_CLOSE;
    private Mat morphKernelMat = null;
    private boolean circleDetectionEnabled = false;
    private double minCircleDistance = 0.0;
    private Size gaussianBlurKernelSize = null;
    private Integer medianBlurKernelSize = null;
    private boolean cannyEdgeEnabled = false;
    private double cannyEdgeThreshold1 = 0.0;
    private double cannyEdgeThreshold2 = 0.0;
    private TrcVisionPerformanceMetrics performanceMetrics = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pipelineParams specifies the pipeline parameters.
     */
    public TrcOpenCvColorBlobPipeline(String instanceName, PipelineParams pipelineParams)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.pipelineParams = pipelineParams;
        intermediateMats = new Mat[NUM_INTERMEDIATE_MATS];
        // Allocate Intermediate Mats, intermediateMats[0] is always the input Mat, no need to allocate.
        for (int i = 1; i < intermediateMats.length; i++)
        {
            intermediateMats[i] = new Mat();
        }
    }   //TrcOpenCvColorBlobPipeline

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
     * This method enables/disables performance metrics.
     *
     * @param enabled specifies true to enable performance metrics, false to disable.
     */
    public void setPerformanceMetricsEnabled(boolean enabled)
    {
        if (performanceMetrics == null && enabled)
        {
            performanceMetrics = new TrcVisionPerformanceMetrics(instanceName);
        }
        else if (performanceMetrics != null && !enabled)
        {
            performanceMetrics = null;
        }
    }   //setPerformanceMetricsEnabled

    /**
     * This method prints the performance metrics to the trace log.
     */
    public void printPerformanceMetrics()
    {
        if (performanceMetrics != null)
        {
            performanceMetrics.printMetrics(tracer);
        }
    }   //printPerformanceMetrics

    /**
     * This method adds another set of color threshold values.
     *
     * @param name specifies the name of the color ranges. This will be used to label the detected object.
     * @param lowThresholds specifies the low threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
     *        {Y, Cr, Cb} etc.)
     * @param highThresholds specifies the high threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
     *        {Y, Cr, Cb} etc.)
     */
    public void addColorThresholds(String name, double[] lowThresholds, double[] highThresholds)
    {
        if (lowThresholds.length != 3)
        {
            throw new IllegalArgumentException("lowThresholds must be an array of 3 doubles.");
        }

        if (highThresholds.length != 3)
        {
            throw new IllegalArgumentException("highThresholds must be an array of 3 doubles.");
        }

        pipelineParams.colorThresholdsList.add(new ColorThresholds(name, lowThresholds, highThresholds));
    }   //addColorThresholds

    /**
     * This method sets the color threshold values set.
     *
     * @param name specifies the name of the color ranges. This will be used to label the detected object.
     * @param lowThresholds specifies the low threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
     *        {Y, Cr, Cb} etc.)
     * @param highThresholds specifies the high threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
     *        {Y, Cr, Cb} etc.)
     */
    public void setColorThresholds(String name, double[] lowThresholds, double[] highThresholds)
    {
        pipelineParams.colorThresholdsList.clear();
        addColorThresholds(name, lowThresholds, highThresholds);
    }   //setColorThresholds

    /**
     * This method checks if the specified color threshold set is enabled.
     *
     * @param name specifies the name of the color threshold set.
     * @return true if found, false otherwise.
     */
    public boolean isColorThresholdsEnabled(String name)
    {
        for (ColorThresholds ct: pipelineParams.colorThresholdsList)
        {
            if (ct.name.equals(name))
            {
                return true;
            }
        }
        return false;
    }   //isColorThresholdsEnabled

    /**
     * This method enables Morphology operation in the pipeline with the specifies kernel shape and size.
     *
     * @param morphOp specifies the Morphology operation.
     * @param kernelShape specifies the kernel shape.
     * @param kernelSize specifies the kernel size.
     */
    public void enableMorphology(int morphOp, int kernelShape, Size kernelSize)
    {
        if (morphKernelMat != null)
        {
            // Release an existing kernel mat if there is one.
            morphKernelMat.release();
        }
        this.morphOp = morphOp;
        morphKernelMat = Imgproc.getStructuringElement(kernelShape, kernelSize);
    }   //enableMorphology

    /**
     * This method enables Morphology operation in the pipeline with default kernel shape and size.
     *
     * @param morphOp specifies the Morphology operation.
     */
    public void enableMorphology(int morphOp)
    {
        enableMorphology(morphOp, Imgproc.MORPH_ELLIPSE, new Size(5, 5));
    }   //enableMorphology

    /**
     * This method enables Morphology operation in the pipeline with default kernel shape and size.
     */
    public void enableMorphology()
    {
        enableMorphology(Imgproc.MORPH_CLOSE, Imgproc.MORPH_ELLIPSE, new Size(5, 5));
    }   //enableMorphology

    /**
     * This method disables Morphology operation in the pipeline.
     */
    public void disableMorphology()
    {
        if (morphKernelMat != null)
        {
            morphKernelMat.release();
            morphKernelMat = null;
        }
    }   //disableMorphology

    /**
     * This method enables circle detection in the pipeline with the given parameters.
     *
     * @param minCircleDistance specifies the minimum distance between detected circle centers.
     */
    public void enableCircleDetection(double minCircleDistance)
    {
        this.minCircleDistance = minCircleDistance;
        this.circleDetectionEnabled = true;
        tracer.traceInfo(instanceName, "Enabling Circle Detection: minCircleDistance=%f", minCircleDistance);
    }   //enableCircleDetection

    /**
     * This method disables circle detection in the pipeline.
     */
    public void disableCircleDetection()
    {
        this.minCircleDistance = 0.0;
        this.circleDetectionEnabled = false;
        tracer.traceInfo(instanceName, "Disabling Circle Detection.");
    }   //disableCircleDetection

    /**
     * This method enables blur operation for circle detection.
     *
     * @param useGaussianBlur specifies true to use Gaussian Blur, false to use Median Blur.
     * @param kernelSize specifies the kernel size in pixels.
     */
    public void enableCircleBlur(boolean useGaussianBlur, int kernelSize)
    {
        tracer.traceInfo(
            instanceName,
            "Enabling Circle Blur (%s): KernelSize=%d", useGaussianBlur? "Gaussian": "Median", kernelSize);
        if (useGaussianBlur)
        {
            gaussianBlurKernelSize = new Size(kernelSize, kernelSize);
            medianBlurKernelSize = null;
        }
        else
        {
            if (kernelSize % 2 == 0 || kernelSize <= 1)
            {
                throw new IllegalArgumentException("kernelSize for Median Blur must be odd and greater than 1.");
            }
            medianBlurKernelSize = kernelSize;
            gaussianBlurKernelSize = null;
        }
    }   //enableCircleBlur

    /**
     * This method enables blur operation for circle detection.
     *
     * @param useGaussianBlur specifies true to use Gaussian Blur, false to use Median Blur.
     */
    public void enableCircleBlur(boolean useGaussianBlur)
    {
        enableCircleBlur(useGaussianBlur, DEF_BLUR_KERNEL_SIZE);
    }   //enableCircleBlur

    /**
     * This method disables blur operation for circle detection.
     */
    public void disableCircleBlur()
    {
        gaussianBlurKernelSize = null;
        medianBlurKernelSize = null;
    }   //disableCircleBlur

    /**
     * This method enables Canny Edge Detection with the specified threshold values.
     *
     * @param threshold1 specifies threshold 1 value.
     * @param threshold2 specifies threshold 2 value.
     */
    public void enableCannyEdgeDetection(double threshold1, double threshold2)
    {
        this.cannyEdgeThreshold1 = threshold1;
        this.cannyEdgeThreshold2 = threshold2;
        this.cannyEdgeEnabled = true;
        tracer.traceInfo(
            instanceName, "Enabling Canny Edge Detection: threshold1=%f, threshold2=%f", threshold1, threshold2);
    }   //enableCannyEdgeDetection

    /**
     * This method disables Canny Edge Detection.
     */
    public void disableCannyEdgeDetection()
    {
        this.cannyEdgeThreshold1 = this.cannyEdgeThreshold2 = 0.0;
        this.cannyEdgeEnabled = false;
        tracer.traceInfo(instanceName, "Disabling Canny Edge Detection.");
    }   //disableCannyEdgeDetection

    /**
     * This method checks if the mat is the same type as expected. If not, it will recreate the mat with the expected
     * type and size.
     *
     * @param mat specifies the mat.
     * @param size specifies the size to recreate.
     * @param cvType specifies the type to recreate.
     */
    private void setExpectedMatType(Mat mat, Size size, int cvType)
    {
        if (mat.type() != cvType)
        {
            mat.create(size, cvType);
        }
    }   //setExpectedMatType

    //
    // Implements TrcOpenCvPipeline interface.
    //

    /**
     * This method is called to reset the state of the pipeline if any.
     */
    @Override
    public void reset()
    {
        if (performanceMetrics != null)
        {
            performanceMetrics.reset();
        }
        intermediateStep = 0;
    }   //reset

    /**
     * This method is called to process the input image through the pipeline.
     *
     * @param input specifies the input image to be processed.
     * @return array of detected objects.
     */
    @Override
    public DetectedObject[] process(Mat input)
    {
        ArrayList<DetectedObject> detectedObjectsList = new ArrayList<>();
        double startTime;
        Mat output;
        int matIndex = 0;

        intermediateMats[matIndex] = input;
        startTime = TrcTimer.getCurrentTime();

        // Do color space conversion.
        if (pipelineParams.colorConversion != null)
        {
            output = intermediateMats[++matIndex];
            setExpectedMatType(output, input.size(), CvType.CV_32FC3);
            Imgproc.cvtColor(input, output, pipelineParams.colorConversion);
            input = output;
        }

        int ctStartMat = matIndex;
        Mat colorConvertedMat = input;
        for (ColorThresholds ct: pipelineParams.colorThresholdsList)
        {
            ArrayList<MatOfPoint> contoursOutput = new ArrayList<>();

            matIndex = ctStartMat;
            input = colorConvertedMat;
            // Do color filtering.
            output = intermediateMats[++matIndex];
            setExpectedMatType(output, input.size(), CvType.CV_8UC1);
            Core.inRange(input, new Scalar(ct.lowThresholds), new Scalar(ct.highThresholds), output);
            input = output;
            // Do morphology.
            if (morphKernelMat != null)
            {
                output = intermediateMats[++matIndex];
                setExpectedMatType(output, input.size(), CvType.CV_8UC1);
                Imgproc.morphologyEx(input, output, morphOp, morphKernelMat);
                input = output;
            }

            if (circleDetectionEnabled)
            {
                // Apply mask to the original image.
                output = intermediateMats[++matIndex];
                setExpectedMatType(output, input.size(), CvType.CV_32FC3);
                output.setTo(new Scalar(0));
                Core.bitwise_and(intermediateMats[0], intermediateMats[0], output, input);
                input = output;
                // Convert masked result to gray.
                output = intermediateMats[++matIndex];
                setExpectedMatType(output, input.size(), CvType.CV_8UC1);
                Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2GRAY);
                input = output;
                // Circle Blur.
                if (gaussianBlurKernelSize != null)
                {
                    output = intermediateMats[++matIndex];
                    setExpectedMatType(output, input.size(), CvType.CV_8UC1);
                    Imgproc.GaussianBlur(input, output, gaussianBlurKernelSize, 2, 2);
                    input = output;
                }
                else if (medianBlurKernelSize != null)
                {
                    output = intermediateMats[++matIndex];
                    setExpectedMatType(output, input.size(), CvType.CV_8UC1);
                    Imgproc.medianBlur(input, output, medianBlurKernelSize);
                    input = output;
                }
                // Hough Circle Detection.
                Mat circles = new Mat();
                Imgproc.HoughCircles(
                    input,
                    circles,
                    Imgproc.CV_HOUGH_GRADIENT,
                    1.0,                // dp (accumulator resolution)
                    minCircleDistance,  // minDist (min distance between circle centers)
                    100.0,              // param1: upper threshold for Canny
                    30.0,               // param2: threshold for center detection (smaller = more circles)
                    (int) (pipelineParams.filterContourParams.widthRange[0]/2.0),    // min radius
                    (int) (pipelineParams.filterContourParams.widthRange[1]/2.0));   // max radius
                // Create contours for detected circles.
                for (int i = 0; i < circles.cols(); i++)
                {
                    double[] data = circles.get(0, i);
                    if (data == null) continue;

                    Point center = new Point(data[0], data[1]);
                    int radius = (int) Math.round(data[2]);

                    // approximate circle with 16-point polygon
                    Point[] pts = new Point[16];
                    for (int j = 0; j < pts.length; j++)
                    {
                        double angle = 2*Math.PI*j/16;
                        pts[j] = new Point(center.x + radius*Math.cos(angle), center.y + radius*Math.sin(angle));
                    }
                    MatOfPoint circleContour = new MatOfPoint();
                    circleContour.fromArray(pts);
                    contoursOutput.add(circleContour);
//                    // Optional: draw on input for visualization
//                    Imgproc.circle(intermediateMats[0], center, radius, new Scalar(255,255,255), 2);
//                    Imgproc.circle(intermediateMats[0], center, 2, new Scalar(255,255,255), -1);
                }
                circles.release();
            }
            // Canny Edge detection is not applicable for circle detection.
            else if (cannyEdgeEnabled)
            {
                output = intermediateMats[++matIndex];
                setExpectedMatType(output, input.size(), CvType.CV_8UC1);
                Imgproc.Canny(input, output, cannyEdgeThreshold1, cannyEdgeThreshold2);
                input = output;
            }
            // Circle Detection creates its own contours.
            if (!circleDetectionEnabled)
            {
                // Find contours.
                Imgproc.findContours(
                    input, contoursOutput, hierarchy, pipelineParams.contourRetrievalMode, Imgproc.CHAIN_APPROX_SIMPLE);
            }
            // Do contour filtering.
            if (pipelineParams.filterContourParams != null)
            {
                ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();
                filterContours(contoursOutput, pipelineParams.filterContourParams, filterContoursOutput);
                contoursOutput.clear();
                contoursOutput.addAll(filterContoursOutput);
            }
            // Process contour result.
            for (MatOfPoint contour: contoursOutput)
            {
                detectedObjectsList.add(new DetectedObject(ct.name, contour));
            }
        }
        if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

        DetectedObject[] detectedObjects = detectedObjectsList.toArray(new DetectedObject[0]);
        detectedObjectsUpdate.set(detectedObjects);

        if (annotateEnabled)
        {
            Mat annotateMat = getIntermediateOutput(intermediateStep);
            Scalar rectColor, textColor;

            if (annotateMat.type() != CvType.CV_8UC1)
            {
                rectColor = ANNOTATE_RECT_COLOR;
                textColor = ANNOTATE_TEXT_COLOR;
            }
            else
            {
                rectColor = ANNOTATE_RECT_WHITE;
                textColor = ANNOTATE_RECT_WHITE;
            }

            if (detectedObjects.length > 0)
            {
                annotateFrame(
                    annotateMat, detectedObjects, drawRotatedRect, drawCrosshair, rectColor, ANNOTATE_RECT_THICKNESS,
                    textColor, ANNOTATE_FONT_SCALE);
                if (pipelineParams.cameraMatrix != null)
                {
                    drawAxes(annotateMat);
                }
            }

            if (drawCrosshair)
            {
                int imageRows = annotateMat.rows();
                int imageCols = annotateMat.cols();
                Imgproc.drawMarker(
                    annotateMat, new Point(imageCols/2.0, imageRows/2.0), rectColor, Imgproc.MARKER_CROSS,
                    Math.max(imageRows, imageCols), ANNOTATE_RECT_THICKNESS);
            }
        }

        return detectedObjects;
    }   //process

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    @Override
    public DetectedObject[] getDetectedObjects()
    {
        return detectedObjectsUpdate.getAndSet(null);
    }   //getDetectedObjects

    /**
     * This method enables image annotation of the detected object.
     *
     * @param drawRotatedRect specifies true to draw rotated rectangle, false to draw bounding rectangle.
     * @param drawCrosshair specifies true to draw crosshair at the center of the screen, false otherwise.
     */
    @Override
    public void enableAnnotation(boolean drawRotatedRect, boolean drawCrosshair)
    {
        this.annotateEnabled = true;
        this.drawRotatedRect = drawRotatedRect;
        this.drawCrosshair = drawCrosshair;
    }   //setAnnotateEnabled

    /**
     * This method disables image annotation.
     */
    @Override
    public void disableAnnotation()
    {
        this.annotateEnabled = false;
        this.drawRotatedRect = false;
        this.drawCrosshair = false;
    }   //disableAnnotation

    /**
     * This method checks if image annotation is enabled.
     *
     * @return true if annotation is enabled, false otherwise.
     */
    @Override
    public boolean isAnnotateEnabled()
    {
        return annotateEnabled;
    }   //isAnnotateEnabled

    /**
     * This method sets the intermediate mat of the pipeline as the video output mat.
     *
     * @param intermediateStep specifies the intermediate mat used as video output (0 is the original input frame).
     */
    @Override
    public void setVideoOutput(int intermediateStep)
    {
        if (intermediateStep >= 0 && intermediateStep < intermediateMats.length)
        {
            this.intermediateStep = intermediateStep;
        }
    }   //setVideoOutput

    /**
     * This method cycles to the next intermediate mat of the pipeline as the video output mat.
     */
    @Override
    public void setNextVideoOutput()
    {
        intermediateStep = (intermediateStep + 1) % intermediateMats.length;
        if (intermediateMats[intermediateStep] == null || intermediateMats[intermediateStep].empty())
        {
            // This mat is empty, skip to the next mat.
            // Warning: this assumes there is at least one non-empty mat in the array. If not, this will become a
            // runaway recursion. (This shouldn't happen because Mat[0] should always be the input mat).
            setNextVideoOutput();
        }
    }   //setNextVideoOutput

    /**
     * This method returns an intermediate processed frame. Typically, a pipeline processes a frame in a number of
     * steps. It may be useful to see an intermediate frame for a step in the pipeline for tuning or debugging
     * purposes.
     *
     * @param step specifies the intermediate step (0 is the original input frame).
     * @return processed frame of the specified step.
     */
    @Override
    public Mat getIntermediateOutput(int step)
    {
        Mat mat = null;

        if (step >= 0 && step < intermediateMats.length)
        {
            mat = intermediateMats[step];
        }

        return mat;
    }   //getIntermediateOutput

    /**
     * This method returns the selected intermediate output Mat.
     *
     * @return selected output mat.
     */
    @Override
    public Mat getSelectedOutput()
    {
        return getIntermediateOutput(intermediateStep);
    }   //getSelectedOutput

    private void drawAxes(Mat img)
    {
        // Length of the axis lines
        double axisLength = 5.0;

        // Define the points in 3D space for the axes
        MatOfPoint3f axisPoints = new MatOfPoint3f(
            new Point3(0, 0, 0),
            new Point3(axisLength, 0, 0),
            new Point3(0, axisLength, 0),
            new Point3(0, 0, -axisLength)); // Z axis pointing away from the camera

        // Project the 3D points to 2D image points
        MatOfPoint2f imagePoints = new MatOfPoint2f();
        Calib3d.projectPoints(
            axisPoints, rvec, tvec, pipelineParams.cameraMatrix, pipelineParams.distCoeffs, imagePoints);

        Point[] imgPts = imagePoints.toArray();

        // Draw the axis lines
        Imgproc.line(img, imgPts[0], imgPts[1], new Scalar(0, 0, 255), 1); // X axis in red
        Imgproc.line(img, imgPts[0], imgPts[2], new Scalar(0, 255, 0), 1); // Y axis in green
        Imgproc.line(img, imgPts[0], imgPts[3], new Scalar(255, 0, 0), 1); // Z axis in blue
    }   //drawAxes

    /**
     * This method filters out contours that do not meet certain criteria.
     *
     * @param inputContours specifies the input list of contours.
     * @param filterContourParams specifies the filter contour parameters.
     * @param output specifies the the output list of contours.
     */
    private void filterContours(
        List<MatOfPoint> inputContours, FilterContourParams filterContourParams, List<MatOfPoint> output)
    {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //
        // Perform the filtering.
        //
        for (int i = 0; i < inputContours.size(); i++)
        {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            // Check width.
            if (bb.width < filterContourParams.widthRange[0] || bb.width > filterContourParams.widthRange[1])
            {
                continue;
            }
            // Check height.
            if (bb.height < filterContourParams.heightRange[0] || bb.height > filterContourParams.heightRange[1])
            {
                continue;
            }
            // Check area.
            final double area = Imgproc.contourArea(contour);
            if (area < filterContourParams.minArea)
            {
                continue;
            }
            // Check perimeter.
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < filterContourParams.minPerimeter)
            {
                continue;
            }
            // Check solidity.
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++)
            {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < filterContourParams.solidityRange[0] || solid > filterContourParams.solidityRange[1])
            {
                continue;
            }
            // Check vertex count.
            if (contour.rows() < filterContourParams.verticesRange[0] ||
                contour.rows() > filterContourParams.verticesRange[1])
            {
                continue;
            }
            // Check aspect ratio.
            final double ratio = bb.width / (double)bb.height;
            if (ratio < filterContourParams.aspectRatioRange[0] || ratio > filterContourParams.aspectRatioRange[1])
            {
                continue;
            }

            output.add(contour);
        }
    }   //filterContours

}  //class TrcOpenCvColorBlobPipeline
