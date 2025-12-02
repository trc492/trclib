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
    public static class Annotation
    {
        public boolean enabled = false;
        public boolean drawRotatedRect = false;
        public boolean drawCrosshair = false;

        public void setAnnotation(boolean enabled, boolean drawRotatedRect, boolean drawCrosshair)
        {
            this.enabled = enabled;
            this.drawRotatedRect = drawRotatedRect;
            this.drawCrosshair = drawCrosshair;
        }   //setAnnotation

        public void setAnnotationEnabled(boolean enabled)
        {
            this.enabled = enabled;
        }   //setAnnotationEnabled

        @Override
        public String toString()
        {
            return "(enabled=" + enabled +
                   ",drawRotatedRect=" + drawRotatedRect +
                   ",drawCrosshair=" + drawCrosshair + ")";
        }   //toString
    }   //class Annotation

    public static class Roi
    {
        public boolean enabled = false;
        public Rect rectRoi = null;

        public Roi(int left, int top, int right, int bottom)
        {
            enabled = true;
            rectRoi = new Rect(left, top, right - left, bottom - top);
        }   //Roi

        @Override
        public String toString()
        {
            return "(enabled=" + enabled +
                   ",rect=" + rectRoi + ")";
        }   //toString
    }   //class Roi

    public enum ColorConversion
    {
        RGBToYCrCb(Imgproc.COLOR_RGB2YCrCb),
        RGBToHSV(Imgproc.COLOR_RGB2HSV);

        public final int value;
        ColorConversion(int value)
        {
            this.value = value;
        }   //ColorConversion
    }   //enum ColorConversion

    /**
     * This class encapsulates color thresholding operation properties of the pipeline.
     */
    public static class ColorThresholds
    {
        // Hide the name field from Dashboard.
        public final String name;
        public boolean enabled = true;
        public double[] lowThresholds = new double[3];
        public double[] highThresholds = new double[3];

        /**
         * Constructor: Create an instance of the object.
         *
         * @param name specifies the name of the color ranges. This will be used to label the detected object.
         * @param enabled specifies true to enable the filter, false to disable.
         * @param lowThresholds specifies the low threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
         *        {Y, Cr, Cb} etc.)
         * @param highThresholds specifies the high threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
         *        {Y, Cr, Cb} etc.)
         */
        public ColorThresholds(String name, boolean enabled, double[] lowThresholds, double[] highThresholds)
        {
            this.name = name;
            this.enabled = enabled;
            this.lowThresholds = lowThresholds;
            this.highThresholds = highThresholds;
        }   //ColorThresholds

        @Override
        public String toString()
        {
            return "(name=" + name +
                   ",enabled=" + enabled +
                   ",lowThresholds=" + Arrays.toString(lowThresholds) +
                   ",highThresholds=" + Arrays.toString(highThresholds) + ")";
        }   //toString
    }   //class ColorThresholds

    public static class Morphology
    {
        public boolean enabled = false;
        public boolean close = true;
        public int kernelSize = 5;
        private int createdKernelSize = 0;
        private Mat kernelMat = null;

        public void setMorphology(boolean enabled, boolean close, int kSize)
        {
            this.enabled = enabled;
            if (enabled)
            {
                this.close = close;
                // kSize was not specified, use default size.
                if (kSize == 0)
                {
                    kSize = 5;
                }

                // We don't have a kernelMat or we are changing its size.
                if (this.kernelMat == null || this.createdKernelSize != kSize)
                {
                    // Create a new kernelMat. If old one exist, release it first.
                    if (kernelMat != null)
                    {
                        kernelMat.release();
                    }
                    this.kernelMat = Imgproc.getStructuringElement(
                        Imgproc.CV_SHAPE_ELLIPSE, new Size(kSize, kSize));
                    this.createdKernelSize = kSize;
                    this.kernelSize = kSize;
                }
            }
            else if (kernelMat != null)
            {
                // Disabling, release the kernelMat.
                this.kernelMat.release();
                this.createdKernelSize = 0;
            }
        }   //setMorphology

        public void refreshKernelMat()
        {
            if (enabled && kernelSize != createdKernelSize)
            {
                // Dashboard has changed kernelSize.
                TrcDbgTrace.globalTraceInfo(
                    "Morphology", "Dashboard has changed kernelSize " + createdKernelSize + "->" + kernelSize);
                setMorphology(true, close, kernelSize);
            }
        }   //refreshKernelMat

        @Override
        public String toString()
        {
            return "(enabled=" + enabled +
                   ",close=" + close +
                   ",createdKernelSize=" + createdKernelSize + ")";
        }   //toString
    }   //class Morphology

    public static class CircleDetection
    {
        public boolean enabled = false;
        public double minCircleDistance = 0.0;

        public void setCircleDetection(boolean enabled, double minCircleDistance)
        {
            this.enabled = enabled;
            this.minCircleDistance = minCircleDistance;
        }   //setCircleDetection

        public void setCircleDetectionEnabled(boolean enabled)
        {
            this.enabled = enabled;
        }   //setCircleDetectionEnabled

        @Override
        public String toString()
        {
            return "(enabled=" + enabled +
                   ",minCircleDistance=" + minCircleDistance + ")";
        }   //toString
    }   //class CircleDetection

    public static class CircleBlur
    {
        public boolean enabled = false;
        public boolean useGaussian = true;
        public int kSize = 9;
        private Size kernelSize = null;

        public void setCircleBlur(boolean enabled, boolean useGaussian, int kSize)
        {
            if (!useGaussian)
            {
                if (kSize <= 1)
                {
                    kSize = 3;
                }
                else if (kSize % 2 == 0)
                {
                    kSize++;
                }
            }
            else if (kSize == 0)
            {
                kSize = 9;
            }

            this.enabled = enabled;
            this.useGaussian = useGaussian;
            this.kernelSize = new Size(kSize, kSize);
            this.kSize = kSize;
        }   //setCircleBlur

        public void refreshKernelSize()
        {
            if (enabled && kernelSize.width != kSize)
            {
                // Dashboard has changed kSize.
                TrcDbgTrace.globalTraceInfo(
                    "CircleBlur", "Dashboard has changed kSize " + kernelSize.width + "->" + kSize);
                setCircleBlur(true, useGaussian, kSize);
            }
        }   //refreshKernelSize

        @Override
        public String toString()
        {
            return "(enabled=" + enabled +
                   ",useGaussian=" + useGaussian +
                   ",kSize=" + kSize +
                   ",kernelSize=" + kernelSize + ")";
        }   //toString
    }   //class CircleBlur

    public static class CannyEdgeDetection
    {
        public boolean enabled = false;
        public double threshold1 = 0.0;
        public double threshold2 = 0.0;

        public void setCannyEdgeDetection(boolean enabled, double threshold1, double threshold2)
        {
            this.enabled = enabled;
            this.threshold1 = threshold1;
            this.threshold2 = threshold2;
        }   //setCannyEdgeDetection

        public void setCannyEdgeDetectionEnabled(boolean enabled)
        {
            this.enabled = enabled;
        }   //setCannyEdgeDetectionEnabled

        @Override
        public String toString()
        {
            return "(enabled=" + enabled +
                   ",threshold1=" + threshold1 +
                   ",threshold2=" + threshold2 + ")";
        }   //toString
    }   //class CannyEdgeDetection
    /**
     * This class encapsulates all the filter contour parameters.
     */
    public static class FilterContourParams
    {
        public boolean filterContourEnabled = false;
        public boolean minAreaPerimeterFilterEnabled = false;
        public double minArea = 0.0;
        public double minPerimeter = 0.0;
        public boolean widthHeightFilterEnabled = false;
        public double[] widthRange = {0.0, 1000.0};
        public double[] heightRange = {0.0, 1000.0};
        public boolean solidityFilterEnabled = false;
        public double[] solidityRange = {0.0, 100.0};
        public boolean verticesFilterEnabled = false;
        public double[] verticesRange = {0.0, 1000000.0};
        public boolean aspectRatioFilterEnabled = false;
        public double[] aspectRatioRange = {0.0, 1000.0};

        public FilterContourParams setMinArea(double minArea)
        {
            this.minAreaPerimeterFilterEnabled = true;
            this.minArea = minArea;
            return this;
        }   //setMinArea

        public FilterContourParams setMinPerimeter(double minPerimeter)
        {
            this.minAreaPerimeterFilterEnabled = true;
            this.minPerimeter = minPerimeter;
            return this;
        }   //setMinPerimeter

        public FilterContourParams setWidthRange(double min, double max)
        {
            this.widthHeightFilterEnabled = true;
            this.widthRange[0] = min;
            this.widthRange[1] = max;
            return this;
        }   //setWidthRange

        public FilterContourParams setHeightRange(double min, double max)
        {
            this.widthHeightFilterEnabled = true;
            this.heightRange[0] = min;
            this.heightRange[1] = max;
            return this;
        }   //setHeightRange

        public FilterContourParams setSolidityRange(double min, double max)
        {
            this.solidityFilterEnabled = true;
            this.solidityRange[0] = min;
            this.solidityRange[1] = max;
            return this;
        }   //setSolidityRange

        public FilterContourParams setVerticesRange(double min, double max)
        {
            this.verticesFilterEnabled = true;
            this.verticesRange[0] = min;
            this.verticesRange[1] = max;
            return this;
        }   //setVerticesRange

        public FilterContourParams setAspectRatioRange(double min, double max)
        {
            this.aspectRatioFilterEnabled = true;
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
            return "(enabled=" + filterContourEnabled +
                   ",minArea=" + minArea +
                   ",minPerim=" + minPerimeter +
                   ",width=(" + widthRange[0] + "," + widthRange[1] + ")" +
                   ",height=(" + heightRange[0] + "," + heightRange[1] + ")" +
                   ",solidity=(" + solidityRange[0] + "," + solidityRange[1] + ")" +
                   ",vertices=(" + verticesRange[0] + "," + verticesRange[1] + ")" +
                   ",aspectRatio=(" + aspectRatioRange[0] + "," + aspectRatioRange[1] + "))";
        }   //toString

    }   //class FilterContourParams

    /**
     * This class contains all the pipeline parameters.
     */
    public static class PipelineParams
    {
        public Annotation annotation = new Annotation();
        public Roi roi = null;
        public ColorConversion colorConversion = null;
        private final ArrayList<ColorThresholds> colorThresholdsList = new ArrayList<>();
        public ColorThresholds[] colorThresholdSets = null;
        public Morphology morphology = new Morphology();
        public CircleDetection circleDetection = new CircleDetection();
        public CircleBlur circleBlur = new CircleBlur();
        public CannyEdgeDetection cannyEdgeDetection = new CannyEdgeDetection();
        public boolean externalContour = true;
        public FilterContourParams filterContourParams = new FilterContourParams();

        /**
         * This method enables annotation.
         *
         * @param drawRotatedRect specifies true to draw rotated rectangle instead of bounding rectangle.
         * @param drawCrosshair specifies true to draw crosshair.
         * @return this object for chaining.
         */
        public PipelineParams setAnnotation(boolean drawRotatedRect, boolean drawCrosshair)
        {
            annotation.setAnnotation(true, drawRotatedRect, drawCrosshair);
            return this;
        }   //setAnnotation

        /**
         * This method sets the Region Of Interest in Unity Center Coordinates.
         *
         * @param left specifies the left coordinate in the range between -1 and 1 from image center.
         * @param top specifies the top coordinate in the range between -1 and 1 from image center.
         * @param right specifies the right coordinate in the range between -1 and 1 from image center.
         * @param bottom specifies the bottom coordinate in the range between -1 and 1 from image center.
         * @param imageWidth specifies the image width.
         * @param imageHeight specifies the image height.
         * @return this object for chaining.
         */
        public PipelineParams setRoi(
            double left, double top, double right, double bottom, int imageWidth, int imageHeight)
        {
            if (left >= right || top >= bottom || left < -1.0 || top < -1.0 || right > 1.0 || bottom > 1.0)
            {
                throw new IllegalArgumentException("Invalid Roi values.");
            }

            left = TrcUtil.scaleRange(left, -1.0, 1.0, 0.0, (double) imageWidth);
            top = TrcUtil.scaleRange(top, -1.0, 1.0, 0.0, (double) imageHeight);
            right = TrcUtil.scaleRange(right, -1.0, 1.0, 0.0, (double) imageWidth);
            bottom = TrcUtil.scaleRange(bottom, -1.0, 1.0, 0.0, (double) imageHeight);
            roi = new Roi((int) left, (int) top, (int) right, (int) bottom);
            return this;
        }   //setRoi

        /**
         * This method sets the Region Of Interest in Image Coordinates.
         *
         * @param left specifies the left Roi in image coordinate
         * @param top specifies the top Roi in image coordinate.
         * @param right specifies the right Roi in image coordinate.
         * @param bottom specifies the bottom Roi in image coordinate.
         * @return this object for chaining.
         */
        public PipelineParams setRoi(int left, int top, int right, int bottom)
        {
            roi = new Roi(left, top, right, bottom);
            return this;
        }   //setRoi

        /**
         * This method sets the Color Conversion for the pipeline.
         *
         * @param colorConversion specifies the color conversion, null if no conversion needed.
         * @return this object for chaining.
         */
        public PipelineParams setColorConversion(ColorConversion colorConversion)
        {
            this.colorConversion = colorConversion;
            return this;
        }   //setColorConversion

        /**
         * This method adds a set of color thresholds. It allows the same pipeline to detect different color blobs.
         *
         * @param name specifies the name of the color ranges. This will be used to label the detected object.
         * @param enabled specifies true to enable this color threshold set, false to disable.
         * @param lowThresholds specifies the low threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
         *        {Y, Cr, Cb} etc.)
         * @param highThresholds specifies the high threshold values of the color space (e.g. {R, G, B}, {H, S, V}, or
         *        {Y, Cr, Cb} etc.)
         * @return this object for chaining.
         */
        public PipelineParams addColorThresholds(
            String name, boolean enabled, double[] lowThresholds, double[] highThresholds)
        {
            if (lowThresholds.length != 3)
            {
                throw new IllegalArgumentException("lowThresholds must be an array of 3 doubles.");
            }

            if (highThresholds.length != 3)
            {
                throw new IllegalArgumentException("highThresholds must be an array of 3 doubles.");
            }

            colorThresholdsList.add(new ColorThresholds(name, enabled, lowThresholds, highThresholds));
            return this;
        }   //addColorThresholds

        /**
         * This method creates the ColorThreshold sets from the ColorThresholdsList. This must be called after adding
         * the last ColorThresholds.
         *
         * @return this object for chaining.
         */
        public PipelineParams buildColorThresholdSets()
        {
            if (colorThresholdSets == null)
            {
                colorThresholdSets = colorThresholdsList.toArray(new ColorThresholds[0]);
            }
            return this;
        }   //buildColorThresholdSets

        /**
         * This method enables morphology and sets its parameters.
         *
         * @param close specifies true to perform morphology CLOSE, false to perform morphology OPEN.
         * @param kernelSize specifies the kernel size (same width and height).
         * @return this object for chaining.
         */
        public PipelineParams setMorphology(boolean close, int kernelSize)
        {
            morphology.setMorphology(true, close, kernelSize);
            return this;
        }   //setMorphology

        /**
         * This method enables circle detection and sets its parameter.
         *
         * @param minCircleDistance specifies the minimum circle distance.
         * @return this object for chaining.
         */
        public PipelineParams setCircleDetection(double minCircleDistance)
        {
            circleDetection.setCircleDetection(true, minCircleDistance);
            return this;
        }   //setCircleDetection

        /**
         * This method enables circle blur and sets its parameter.
         *
         * @param useGaussian specifies true to use Gaussian Blur, false to use Median Blur.
         * @param kernelSize specifies the kernel size (same width and height).
         * @return this object for chaining.
         */
        public PipelineParams setCircleBlur(boolean useGaussian, int kernelSize)
        {
            circleBlur.setCircleBlur(true, useGaussian, kernelSize);
            return this;
        }   //setCircleBlur

        /**
         * This method enables circle blur and sets its parameter.
         *
         * @param threshold1 specifies threshold 1 value.
         * @param threshold2 specifies threshold 2 value.
         * @return this object for chaining.
         */
        public PipelineParams setCannyEdgeDetection(double threshold1, double threshold2)
        {
            cannyEdgeDetection.setCannyEdgeDetection(true, threshold1, threshold2);
            return this;
        }   //setCannyEdgeDetection

        /**
         * This method sets contour detection parameters.
         *
         * @param filterParams specifies the contour filtering parameters.
         * @return this object for chaining.
         */
        public PipelineParams setFilterContourParams(boolean external, FilterContourParams filterParams)
        {
            externalContour = external;
            filterContourParams = filterParams;
            filterParams.filterContourEnabled = true;
            return this;
        }   //setFilterContourParams


        @Override
        public String toString()
        {
            return "\tannotation=" + annotation +
                   "\n\tRoi=" + roi +
                   "\n\tcolorConversion=" + colorConversion +
                   "\n\tcolorThresholds=" + Arrays.toString(colorThresholdSets) +
                   "\n\tmorphology=" + morphology +
                   "\n\tcircleDetection=" + circleDetection +
                   "\n\tcircleBlur=" + circleBlur +
                   "\n\tcannyEdge=" + cannyEdgeDetection +
                   "\n\tfilterParams=" + filterContourParams;
        }   //toString

    }   //class PipelineParams

    public static class SolvePnpParams
    {
        private double objWidth = 0.0;
        private double objHeight = 0.0;
        private Mat cameraMatrix = null;
        private MatOfDouble distCoeffs = null;
        private TrcPose3D cameraPose = null;
        private MatOfPoint3f objPoints = null;

        /**
         * This method sets the detected object's real world size.
         *
         * @param objWidth specifies the detected object's real world width.
         * @param objHeight specifies the detected object's real world height.
         * @return this object for chaining.
         */
        public SolvePnpParams setObjectSize(double objWidth, double objHeight)
        {
            this.objWidth = objWidth;
            this.objHeight = objHeight;
            return this;
        }   //setObjectSize

        /**
         * This method sets the parameters used for SolvePnp operation.
         *
         * @param lensInfo specifies the camera lens properties.
         * @param cameraPose specifies the camera pose from robot center.
         * @return this object for chaining.
         */
        public SolvePnpParams setSolvePnpParams(TrcOpenCvDetector.LensInfo lensInfo, TrcPose3D cameraPose)
        {
            cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
            cameraMatrix.put(
                0, 0,
                lensInfo.fx,    0,              lensInfo.cx,
                0,              lensInfo.fy,    lensInfo.cy,
                0,              0,              1);
            this.distCoeffs = new MatOfDouble(lensInfo.distCoeffs);
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
            return "\tobjWidth=" + objWidth +
                   "\n\tobjHeight=" + objHeight +
                   "\n\tcameraPose=" + cameraPose;
        }   //toString

    }   //class SolvePnpParams

    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public class DetectedObject extends TrcOpenCvDetector.DetectedObject<Point[]>
    {
        public final Rect objRect;
        public final RotatedRect rotatedRect;
        public final double rotatedRectAngle;
        public final double objArea;
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
            super(label, null);
            objRect = Imgproc.boundingRect(contour);
            MatOfPoint2f contourPoints2f = new MatOfPoint2f(contour.toArray());
            rotatedRect = Imgproc.minAreaRect(contourPoints2f);
            contourPoints2f.release();
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
            objArea = Imgproc.contourArea(contour);
            // Get the 2D image points from the detected rectangle corners
            rotatedRect.points(vertices);

            // Solve PnP: assuming the object is a rectangle with known dimensions.
            if (solvePnpParams == null || solvePnpParams.cameraMatrix == null || solvePnpParams.distCoeffs != null)
            {
                // Caller did not provide camera matrix nor distortion coefficients so we can't call solvePnP.
                objPose = null;
            }
            else
            {
                MatOfPoint2f verticePoints = new MatOfPoint2f(orderPoints(vertices));
                if (Calib3d.solvePnP(
                    // Define the 3D coordinates of the object corners in the object coordinate space
                    solvePnpParams.objPoints,   // Object points in 3D
                    verticePoints,              // Corresponding image points
                    solvePnpParams.cameraMatrix,
                    solvePnpParams.distCoeffs,
                    rvec,
                    tvec))
                {
                    objPose = projectPose(rvec, tvec, solvePnpParams.cameraPose);
//                    objPose = new TrcPose2D(tvec.get(0, 0)[0], tvec.get(2, 0)[0], -(Math.toDegrees(rvec.get(1, 0)[0])));
                }
                else
                {
                    // SolvePnP failed, return null so caller will determine object pose by Homography.
                    objPose = null;
                }
                verticePoints.release();
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
            return objRect;
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
            return objArea;
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
            return solvePnpParams != null? solvePnpParams.objWidth: null;
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
    private static final int NUM_INTERMEDIATE_MATS = 8;

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final PipelineParams pipelineParams;
    private final SolvePnpParams solvePnpParams;
    private final Mat[] intermediateMats;
    private final Mat circlesMat = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat rvec = new Mat();
    private final Mat tvec = new Mat();

    private final Annotation rawAnnotation = new Annotation();
    private final AtomicReference<DetectedObject[]> detectedObjectsUpdate = new AtomicReference<>();
    private int intermediateStep = 0;
    private TrcVisionPerformanceMetrics performanceMetrics = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pipelineParams specifies pipeline parameters.
     * @param solvePnpParams specifies SolvePnP parameters, can be null if not provided.
     */
    public TrcOpenCvColorBlobPipeline(String instanceName, PipelineParams pipelineParams, SolvePnpParams solvePnpParams)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.pipelineParams = pipelineParams;
        this.solvePnpParams = solvePnpParams;
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
     * @param name specifies the name of the color thresholds set.
     * @param enabled specifies true to enable this color threshold set, false to disable.
     */
    public void setColorThresholdsEnabled(String name, boolean enabled)
    {
        synchronized (pipelineParams)
        {
            for (ColorThresholds ct: pipelineParams.colorThresholdsList)
            {
                if (ct.name.equals(name))
                {
                    ct.enabled = enabled;
                }
            }
        }
    }   //setColorThresholdsEnabled

    /**
     * This method checks if the specified color threshold set is enabled.
     *
     * @param name specifies the name of the color threshold set.
     * @return true if found, false otherwise.
     */
    public boolean isColorThresholdsEnabled(String name)
    {
        synchronized (pipelineParams)
        {
            for (ColorThresholds ct : pipelineParams.colorThresholdsList)
            {
                if (ct.name.equals(name))
                {
                    return ct.enabled;
                }
            }
        }

        return false;
    }   //isColorThresholdsEnabled

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

    private boolean isSubmatrix(Mat mat, Mat submat)
    {
        boolean isSubmat = false;

        if (!mat.empty() && !submat.empty())
        {
            long matPointer = mat.dataAddr();
            long submatPointer = submat.dataAddr();
            if (mat.isContinuous() && mat.elemSize() == submat.elemSize() &&
                mat.rows() >= submat.rows() && mat.cols() >= submat.cols() &&
                submatPointer >= matPointer &&
                submatPointer < matPointer + mat.total() * mat.elemSize())
            {
                isSubmat = true;
            }
        }

        return isSubmat;
    }   //isSubmatrix

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
        DetectedObject[] detectedObjects;
        double startTime;
        Mat output;
        int matIndex = 0;

        synchronized (pipelineParams)
        {
            // Original camera image is CV_8UC4.
            intermediateMats[matIndex] = input;
            startTime = TrcTimer.getCurrentTime();

            // Do ROI.
            if (pipelineParams.roi != null && pipelineParams.roi.enabled)
            {
                Mat mask = Mat.zeros(input.size(), CvType.CV_8UC1);
                Imgproc.rectangle(mask, pipelineParams.roi.rectRoi, new Scalar(255), -1);
                output = intermediateMats[++matIndex];
                setExpectedMatType(output, input.size(), CvType.CV_8UC3);
                input.copyTo(output, mask);
                mask.release();
                tracer.traceDebug(
                    instanceName, "[%d] Roi: type=0x%02x, roiRect=%s",
                    matIndex, output.type(), pipelineParams.roi.rectRoi);
                input = output;
//                // Check if Dashboard has changed ROI rect.
//                pipelineParams.roi.refreshRoi();
//                output = intermediateMats[++matIndex];
//                setExpectedMatType(output, input.size(), CvType.CV_8UC3);
//                input.copyTo(output, pipelineParams.roi.roiMask);

//                output = input.submat(pipelineParams.roi.rectRoi);
//                if (intermediateMats[++matIndex] != null)
//                {
//                    // Releasing the original pre-allocated intermediate Mat or the previous frame's submat.
//                    intermediateMats[matIndex].release();
//                }
//                intermediateMats[matIndex] = output;
//                tracer.traceDebug(
//                    instanceName, "[%d] Roi: type=0x%02x, roiRect=%s",
//                    matIndex, output.type(), pipelineParams.roi.rectRoi);
//                input = output;
            }
            else if (isSubmatrix(intermediateMats[matIndex], intermediateMats[matIndex + 1]))
            {
                // Next mat is a submat of the input. This means the user just disabled ROI.
                // In this case, we need to release the submat and re-allocate a new mat for the next step.
                intermediateMats[matIndex + 1].release();
                intermediateMats[matIndex + 1] = new Mat();
            }
            // Do color space conversion.
            if (pipelineParams.colorConversion != null)
            {
                output = intermediateMats[++matIndex];
                setExpectedMatType(output, input.size(), CvType.CV_8UC3);
                tracer.traceDebug(
                    instanceName, "[%d] cvtColor: type=0x%02x, colorConv=%s",
                    matIndex, output.type(), pipelineParams.colorConversion);
                Imgproc.cvtColor(input, output, pipelineParams.colorConversion.value);
                input = output;
            }

            int ctStartMat = matIndex;
            Mat colorConvertedMat = input;
            if (pipelineParams.colorThresholdSets == null)
            {
                pipelineParams.buildColorThresholdSets();
            }
            for (ColorThresholds ct : pipelineParams.colorThresholdSets)
            {
                tracer.traceDebug(instanceName, "ColorThreshold: colorThreshold=%s", ct);
                if (!ct.enabled) continue;

                ArrayList<MatOfPoint> contoursOutput = new ArrayList<>();

                matIndex = ctStartMat;
                input = colorConvertedMat;
                // Do color filtering.
                output = intermediateMats[++matIndex];
                setExpectedMatType(output, input.size(), CvType.CV_8UC1);
                tracer.traceDebug(instanceName, "[%d] inRange: type0x%02x", matIndex, output.type());
                Core.inRange(input, new Scalar(ct.lowThresholds), new Scalar(ct.highThresholds), output);
                input = output;
                // Do morphology.
                if (pipelineParams.morphology.enabled)
                {
                    // Check if Dashboard has changed Kernel Size.
                    pipelineParams.morphology.refreshKernelMat();
                    if (pipelineParams.morphology.createdKernelSize > 1)
                    {
                        output = intermediateMats[++matIndex];
                        setExpectedMatType(output, input.size(), CvType.CV_8UC1);
                        tracer.traceDebug(
                            instanceName, "[%d] morphology: type=0x%02x, morphology=%s",
                            matIndex, output.type(), pipelineParams.morphology);
                        Imgproc.morphologyEx(
                            input, output, pipelineParams.morphology.close ? Imgproc.MORPH_CLOSE : Imgproc.MORPH_OPEN,
                            pipelineParams.morphology.kernelMat);
                        input = output;
                    }
                }

                if (pipelineParams.circleDetection.enabled)
                {
                    tracer.traceDebug(instanceName, "CircleDetection: %s", pipelineParams.circleDetection);
                    // Apply mask to the original image.
                    output = intermediateMats[++matIndex];
                    setExpectedMatType(output, input.size(), CvType.CV_8UC4);
                    output.setTo(new Scalar(0));
                    tracer.traceDebug(instanceName, "[%d] circleMask: type=0x%02x", matIndex, output.type());
                    Core.bitwise_and(intermediateMats[0], intermediateMats[0], output, input);
                    input = output;
                    // Convert masked result to gray.
                    output = intermediateMats[++matIndex];
                    setExpectedMatType(output, input.size(), CvType.CV_8UC1);
                    tracer.traceDebug(instanceName, "[%d] convertToGray: type=0x%02x", matIndex, output.type());
                    Imgproc.cvtColor(input, output, Imgproc.COLOR_RGBA2GRAY);
                    input = output;
                    // Circle Blur.
                    if (pipelineParams.circleBlur.enabled)
                    {
                        // Check if Dashboard has changed Kernel Size.
                        pipelineParams.circleBlur.refreshKernelSize();
                        output = intermediateMats[++matIndex];
                        setExpectedMatType(output, input.size(), CvType.CV_8UC1);
                        tracer.traceDebug(
                            instanceName, "[%d] blur: type=0x%02x, blur=%s",
                            matIndex, output.type(), pipelineParams.circleBlur);
                        if (pipelineParams.circleBlur.useGaussian)
                        {
                            Imgproc.GaussianBlur(input, output, pipelineParams.circleBlur.kernelSize, 2, 2);
                        }
                        else
                        {
                            Imgproc.medianBlur(input, output, pipelineParams.circleBlur.kSize);
                        }
                        input = output;
                    }
                    // Hough Circle Detection.
                    Imgproc.HoughCircles(
                        input,
                        circlesMat,
                        Imgproc.CV_HOUGH_GRADIENT,
                        1.0,                // dp (accumulator resolution)
                        pipelineParams.circleDetection.minCircleDistance,   // min distance between circle centers
                        100.0,              // param1: upper threshold for Canny
                        30.0,               // param2: threshold for center detection (smaller = more circles)
                        (int) (pipelineParams.filterContourParams.widthRange[0]/2.0),    // min radius
                        (int) (pipelineParams.filterContourParams.widthRange[1]/2.0));   // max radius
                    // Create contours for detected circles.
                    for (int i = 0; i < circlesMat.cols(); i++)
                    {
                        double[] data = circlesMat.get(0, i);
                        if (data == null) continue;

                        Point center = new Point(data[0], data[1]);
                        int radius = (int) Math.round(data[2]);

                        tracer.traceDebug(instanceName, "[circle %d] center=%s, radius=%d", i, center, radius);
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
//                        // Optional: draw on input for visualization
//                        Imgproc.circle(intermediateMats[0], center, radius, new Scalar(255,255,255), 2);
//                        Imgproc.circle(intermediateMats[0], center, 2, new Scalar(255,255,255), -1);
                    }
                }
                // Canny Edge detection is not applicable for circle detection.
                else if (pipelineParams.cannyEdgeDetection.enabled)
                {
                    output = intermediateMats[++matIndex];
                    setExpectedMatType(output, input.size(), CvType.CV_8UC1);
                    tracer.traceDebug(
                        instanceName, "[%d] cannyEdge: type=0x%02x, cannyEdge=%s",
                        matIndex, output.type(), pipelineParams.cannyEdgeDetection);
                    Imgproc.Canny(
                        input, output, pipelineParams.cannyEdgeDetection.threshold1,
                        pipelineParams.cannyEdgeDetection.threshold2);
                    input = output;
                }
                // Circle Detection creates its own contours.
                if (!pipelineParams.circleDetection.enabled)
                {
                    // Find contours.
                    tracer.traceDebug(
                        instanceName, "findContour: type=0x%02x, external=%s",
                        output.type(), pipelineParams.externalContour);
                    Imgproc.findContours(
                        input, contoursOutput, hierarchy,
                        pipelineParams.externalContour? Imgproc.RETR_EXTERNAL: Imgproc.RETR_LIST,
                        Imgproc.CHAIN_APPROX_SIMPLE);
                }
                // Do contour filtering.
                if (pipelineParams.filterContourParams != null &&
                    pipelineParams.filterContourParams.filterContourEnabled)
                {
                    ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();
                    tracer.traceDebug(
                        instanceName, "filterContour: filter=%s", pipelineParams.filterContourParams);
                    filterContours(contoursOutput, pipelineParams.filterContourParams, filterContoursOutput);
                    contoursOutput.clear();
                    contoursOutput.addAll(filterContoursOutput);
                }
                // Process contour result.
                for (MatOfPoint contour : contoursOutput)
                {
                    detectedObjectsList.add(new DetectedObject(ct.name, contour));
                    contour.release();
                }
                tracer.traceDebug(instanceName, "DetectedObj: num=%d", detectedObjectsList.size());
            }
            if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

            detectedObjects = detectedObjectsList.toArray(new DetectedObject[0]);
            detectedObjectsUpdate.set(detectedObjects);

            if (rawAnnotation.enabled)
            {
                Mat annotateMat = getIntermediateOutput(intermediateStep);
                Scalar rectColor, textColor;

                if (annotateMat.channels() > 1)
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
                        annotateMat, detectedObjects, rawAnnotation.drawRotatedRect,
                        rawAnnotation.drawCrosshair, rectColor, ANNOTATE_RECT_THICKNESS, textColor,
                        ANNOTATE_FONT_SCALE);
                    if (solvePnpParams != null && solvePnpParams.cameraMatrix != null)
                    {
                        drawAxes(annotateMat);
                    }
                }

                if (rawAnnotation.drawCrosshair)
                {
                    int imageRows = annotateMat.rows();
                    int imageCols = annotateMat.cols();
                    Imgproc.drawMarker(
                        annotateMat, new Point(imageCols/2.0, imageRows/2.0), rectColor, Imgproc.MARKER_CROSS,
                        Math.max(imageRows, imageCols), ANNOTATE_RECT_THICKNESS);
                }
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
        synchronized (rawAnnotation)
        {
            rawAnnotation.enabled = true;
            rawAnnotation.drawRotatedRect = drawRotatedRect;
            rawAnnotation.drawCrosshair = drawCrosshair;
        }
    }   //enableAnnotation

    /**
     * This method disables image annotation.
     */
    @Override
    public void disableAnnotation()
    {
        synchronized (rawAnnotation)
        {
            rawAnnotation.enabled = false;
            rawAnnotation.drawRotatedRect = false;
            rawAnnotation.drawCrosshair = false;
        }
    }   //disableAnnotation

    /**
     * This method checks if image annotation is enabled.
     *
     * @return true if annotation is enabled, false otherwise.
     */
    @Override
    public boolean isAnnotateEnabled()
    {
        synchronized (rawAnnotation)
        {
            return rawAnnotation.enabled;
        }
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
            axisPoints, rvec, tvec, solvePnpParams.cameraMatrix, solvePnpParams.distCoeffs, imagePoints);

        Point[] imgPts = imagePoints.toArray();

        // Draw the axis lines
        Imgproc.line(img, imgPts[0], imgPts[1], new Scalar(0, 0, 255), 1); // X axis in red
        Imgproc.line(img, imgPts[0], imgPts[2], new Scalar(0, 255, 0), 1); // Y axis in green
        Imgproc.line(img, imgPts[0], imgPts[3], new Scalar(255, 0, 0), 1); // Z axis in blue

        axisPoints.release();
        imagePoints.release();
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
            if (filterContourParams.widthHeightFilterEnabled)
            {
                if (bb.width < filterContourParams.widthRange[0] || bb.width > filterContourParams.widthRange[1])
                {
                    contour.release();
                    continue;
                }
                // Check height.
                if (bb.height < filterContourParams.heightRange[0] || bb.height > filterContourParams.heightRange[1])
                {
                    contour.release();
                    continue;
                }
            }
            // Check area.
            Double area = null;
            if (filterContourParams.minAreaPerimeterFilterEnabled)
            {
                area = Imgproc.contourArea(contour);
                if (area < filterContourParams.minArea)
                {
                    contour.release();
                    continue;
                }
                // Check perimeter.
                if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < filterContourParams.minPerimeter)
                {
                    contour.release();
                    continue;
                }
            }
            // Check solidity.
            if (filterContourParams.solidityFilterEnabled)
            {
                if (area == null)
                {
                    area = Imgproc.contourArea(contour);
                }
                Imgproc.convexHull(contour, hull);
                MatOfPoint mopHull = new MatOfPoint();
                mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
                for (int j = 0; j < hull.size().height; j++)
                {
                    int index = (int) hull.get(j, 0)[0];
                    double[] point = new double[]{contour.get(index, 0)[0], contour.get(index, 0)[1]};
                    mopHull.put(j, 0, point);
                }
                final double solid = 100*area/Imgproc.contourArea(mopHull);
                if (solid < filterContourParams.solidityRange[0] || solid > filterContourParams.solidityRange[1])
                {
                    contour.release();
                    continue;
                }
            }
            // Check vertex count.
            if (filterContourParams.verticesFilterEnabled)
            {
                if (contour.rows() < filterContourParams.verticesRange[0] ||
                    contour.rows() > filterContourParams.verticesRange[1])
                {
                    contour.release();
                    continue;
                }
            }
            // Check aspect ratio.
            if (filterContourParams.aspectRatioFilterEnabled)
            {
                final double ratio = bb.width/(double) bb.height;
                if (ratio < filterContourParams.aspectRatioRange[0] || ratio > filterContourParams.aspectRatioRange[1])
                {
                    contour.release();
                    continue;
                }
            }

            output.add(contour);
        }
    }   //filterContours

}  //class TrcOpenCvColorBlobPipeline
