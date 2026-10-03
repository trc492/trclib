/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
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

import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.Arrays;
import java.util.Locale;

import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcDbgTrace;

/**
 * This class contains platform independent Vision information.
 */
public class TrcVision
{
    /**
     * This class contains the Camera Lens info used by OpenCV SolvePnP.
     */
    public static class LensInfo
    {
        public double fx;
        public double fy;
        public double cx;
        public double cy;
        public double[] distCoeffs;

        /**
         * This method sets the camera lens focal length and principal point.
         *
         * @param fx specifies the focal length in x.
         * @param fy specifies the focal length in y.
         * @param cx specifies the principal point in x.
         * @param cy specifies the principal point in y.
         * @return this object for chaining.
         */
        public LensInfo setLensProperties(double fx, double fy, double cx, double cy)
        {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            return this;
        }   //setLensProperties

        /**
         * This method sets the camera lens distortion coefficients.
         *
         * @param distCoeffs specifies an array containing the lens distortion coefficients.
         * @return this object for chaining.
         */
        public LensInfo setDistortionCoefficents(double... distCoeffs)
        {
            this.distCoeffs = distCoeffs;
            return this;
        }   //setDistortionCoefficients

        @Override
        public String toString()
        {
            return "(fx=" + fx + ",fy=" + fy + ",cx=" + cx + ",cy=" + cy +
                ",distCoeffs=" + Arrays.toString(distCoeffs) + ")";
        }   //toString
    }   //class LensInfo

    /**
     * This class contains camera information.
     */
    public static class CameraInfo
    {
        public String camName = null;
        public int camImageWidth = 0, camImageHeight = 0;
        public Double camHFov = null, camVFov = null;
        public LensInfo lensInfo = null;
        public TrcPose3D camPose = null;
        public TrcHomographyMapper.Rectangle cameraRect = null;
        public TrcHomographyMapper.Rectangle worldRect = null;

        /**
         * This method sets the basic camera info.
         *
         * @param name specifies the name of the camera.
         * @param imageWidth specifies the camera horizontal resolution in pixels.
         * @param imageHeight specifies the camera vertical resolution in pixels.
         * @return this object for chaining.
         */
        public CameraInfo setCameraInfo(String name, int imageWidth, int imageHeight)
        {
            this.camName = name;
            this.camImageWidth = imageWidth;
            this.camImageHeight = imageHeight;
            return this;
        }   //setCameraInfo

        /**
         * This method sets the camera's Field Of View.
         *
         * @param hFov specifies the horizontal field of view in degreees.
         * @param vFov specifies the vertical field of view in degrees.
         * @return this object for chaining.
         */
        public CameraInfo setCameraFOV(double hFov, double vFov)
        {
            this.camHFov = hFov;
            this.camVFov = vFov;
            return this;
        }   //setCameraFOV

        /**
         * This method sets the camera lens properties for SolvePnP.
         *
         * @param lensInfo specifies the camera lens properties.
         * @return this object for chaining.
         */
        public CameraInfo setLensProperties(LensInfo lensInfo)
        {
            this.lensInfo = lensInfo;
            return this;
        }   //setLensProperties

        /**
         * This method sets the camera lens properties for SolvePnP.
         *
         * @param fx specifies the focal length in x.
         * @param fy specifies the focal length in y.
         * @param cx specifies the principal point in x.
         * @param cy specifies the principal point in y.
         * @param distCoeffs specifies an array containing the lens distortion coefficients.
         * @return this object for chaining.
         */
        public CameraInfo setLensProperties(double fx, double fy, double cx, double cy, double[] distCoeffs)
        {
            setLensProperties(
                new LensInfo().setLensProperties(fx, fy, cx, cy).setDistortionCoefficents(distCoeffs));
            return this;
        }   //setLensProperties

        /**
         * This method sets the camera location relative to robot center on the ground.
         *
         * @param xOffset specifies the X offset from robot center (positive right).
         * @param yOffset specifies the Y offset from robot center (positive forward).
         * @param zOffset specifies the Z offset from the ground (positive up).
         * @param pitch specifies pitch angle from horizontal (positive up).
         * @param roll specifies roll angle from vertical (positive left wing up).
         * @param yaw specifies yaw angle from robot forward (positive clockwise).
         * @return this object for chaining.
         */
        public CameraInfo setCameraPose(
            double xOffset, double yOffset, double zOffset, double pitch, double roll, double yaw)
        {
            this.camPose = new TrcPose3D(xOffset, yOffset, zOffset, pitch, roll, yaw);
            return this;
        }   //setCameraPose

        public CameraInfo setHomographyParams(
            TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect)
        {
            this.cameraRect = cameraRect;
            this.worldRect = worldRect;
            return this;
        }   //setHomographyParams
    }   //class CameraInfo

    public interface TargetKnownWidth
    {
        /**
         * This method is called to get the target's real world width so that vision can accurately calculate the
         * target position from the camera.
         *
         * @param targetType specifies the detected target type.
         * @return target real world width in inches, null if not supported or unknown.
         */
        Double getRealWorldWidth(Object targetType);
    }   //interface TargetKnownWidth

    public interface TargetGroundOffset
    {
        /**
         * This method is called to get the target offset from ground so that vision can accurately calculate the
         * target position from the camera.
         *
         * @param targetType specifies the detected target type.
         * @return target ground offset in inches.
         */
        double getOffset(Object targetType);
    }   //interface TargetGroundOffset

    /**
     * This interface provides a method for filtering false positive objects in the detected target list.
     */
    public interface FilterTarget
    {
        /**
         * This method is called to validate the given target as valid for filtering.
         *
         * @param target specifies the target to be validated.
         * @param context specifies the context object passed back to the caller.
         * @return true if the target is valid, false otherwise.
         */
        boolean validateTarget(TargetInfo target, Object context);
    }   //interface FilterTarget

    /**
     * This class encapsulates common vision detected target info. This class is intended to be extended by specific
     * vision detection that may provide additional vision target info.
     */
    public static abstract class TargetInfo
    {
        //
        // Abstract methods provided by subclass.
        //

        /**
         * This method returns the robot field pose on the ground.
         *
         * @param targetFieldPose specifies 3D target field pose, can be null if not provided in which case the
         *                        vision library has built-in Target field poses that calculates robotPose. If
         *                        provided, this method will use it to calculate robot pose.
         * @return robot field pose.
         */
        public abstract TrcPose2D getRobotPose(TrcPose3D targetFieldPose);

        /**
         * This method returns the projected 2D pose on the ground of the detected target relative to the robot center.
         *
         * @return pose of the detected target relative to camera, null if not supported.
         */
        public abstract TrcPose2D getTargetPose();

        /**
         * This method returns the target's real world ground distance from the camera.
         *
         * @return target real world ground distance, null if not supported.
         */
        public abstract Double getTargetDistance();

        /**
         * This method returns the target's real world width.
         *
         * @return target real world width, null if not supported.
         */
        public abstract Double getTargetWidth();

        /**
         * This method returns the normalized area percent of the detected target.
         *
         * @return normalized area percent of the detected target (0.0 to 1.0), null if not supported.
         */
        public abstract Double getNormalizedTargetArea();

        /**
         * This method returns the pixel rect of the detected target.
         *
         * @return pixel rect of the detected target, null if not supported.
         */
        public abstract Rect getPixelRect();

        /**
         * This method returns the pixel width of the detected target. This may be different from pixel rect width.
         * If the target is rotated, this will give you a more accurate width.
         *
         * @return target pixel width, null if not supported.
         */
        public abstract Double getPixelWidth();

        /**
         * This method returns the pixel height of the detected target. This may be different from pixel rect height.
         * If the target is rotated, this will give you a more accurate height.
         *
         * @return target pixel height, null if not supported.
         */
        public abstract Double getPixelHeight();

        /**
         * This method returns the target's rotated rectangle angle.
         *
         * @return rotated rectangle angle, null if not supported.
         */
        public abstract Double getRotatedRectAngle();

        /**
         * This method returns the rotated rect vertices of the detected target.
         *
         * @return rotated rect vertices, null if not supported.
         */
        public abstract Point[] getRotatedRectVertices();

        //
        // Detected target info.
        //

        public final String label;
        protected final CameraInfo cameraInfo;
        protected TrcPose2D robotPose = null;
        protected TrcPose3D targetPose3d = null;
        protected TrcPose2D targetPose2d = null;
        protected Double targetDistance = null;
        protected Double targetWidth = null;
        protected Double normalizedTargetArea = null;
        protected Rect pixelRect = null;
        protected Double pixelWidth = null;
        protected Double pixelHeight = null;
        protected Double rotatedRectAngle = null;
        protected Point[] rotatedRectVertices = null;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param label specifies the target label.
         * @param cameraInfo specifies camera information.
         */
        public TargetInfo(String label, CameraInfo cameraInfo)
        {
            this.label = label;
            this.cameraInfo = cameraInfo;
        }   //TargetInfo

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "label=%s,robotPose=%s,targetPose2d=%s,target(dist=%.1f,width=%.1f,area=%.3f)" +
                ",pixelRect=%s(w=%.0f,h=%.0f),rotatedRectAngle=%.1f",
                label, getRobotPose(null), getTargetPose(), getTargetDistance(), getTargetWidth(),
                getNormalizedTargetArea(), getPixelRect(), getPixelWidth(), getPixelHeight(), getRotatedRectAngle());
        }   //toString

        /**
         * This method returns the projected 3D pose on the ground of the detected target relative to the robot center.
         *
         * @return pose of the detected target relative to camera, null if not supported.
         */
        public TrcPose3D getTargetPose3d()
        {
            if (targetPose3d != null)
            {
                // getTargetPose will calculate targetPose3d if appropriate.
                getTargetPose();
            }

            return targetPose3d;
        }   //getTargetPose3d

        /**
         * This method is called to compute robot pose with the given target field pose.
         *
         * @param targetFieldPose specifies 3D field pose of the target.
         * @return calculated robot pose.
         */
        protected TrcPose2D getRobotPoseByTargetFieldPose(TrcPose3D targetFieldPose)
        {
            TrcPose2D pose = null;
            // Extract your verified local relative target pose (Robot Space, projected to floor)
            TrcPose2D relTarget2d = getTargetPose();

            if (relTarget2d != null)
            {
                TrcPose2D targetField2d = targetFieldPose.toTrcPose2D();
                TrcPose2D unnormalizedRobotPose = targetField2d.addRelativePose(relTarget2d.inverse());
                // Wrap and normalize heading between [-180, 180] degrees
                double normalizedYaw = (unnormalizedRobotPose.angle + 180.0) % 360.0;
                if (normalizedYaw < 0) normalizedYaw += 360.0;
                normalizedYaw -= 180.0;

                pose = new TrcPose2D(unnormalizedRobotPose.x, unnormalizedRobotPose.y, normalizedYaw);
            }

            return pose;
        }   //getRobotPoseByTargetFieldPose

        /**
         * This method calculates the detected target pose by determining the pixel to real world scale using the
         * known width of the detected target.
         *
         * @param knownWidth specifies the known width of the target in real world unit.
         * @return calculated target pose.
         */
        protected TrcPose2D getTargetPoseByKnownWidth(double knownWidth)
        {
            if (pixelRect == null)
            {
                getPixelRect();
            }

            if (pixelRect != null)
            {
                // Angular tracking using calibrated intrinsics (cx, cy, fx, fy)
                double targetXPixel = (pixelRect.x + pixelRect.width / 2.0) - cameraInfo.lensInfo.cx;
                double bearingRad = Math.atan(targetXPixel / cameraInfo.lensInfo.fx);
                double bearingDeg = Math.toDegrees(bearingRad);
                // pixelWidth / xFocalLength = knownWidth / distance
                // => distance = knownWidth * xFocalLength / pixelWidth
                targetDistance = (knownWidth * cameraInfo.lensInfo.fx) / pixelRect.width;
                targetPose2d = new TrcPose2D(
                    targetDistance * Math.sin(bearingRad),
                    targetDistance * Math.cos(bearingRad),
                    bearingDeg);
            }

            return targetPose2d;
        }   //getTargetPoseByKnownWidth

        /**
         * This method calculates the detected target pose using Homography.
         *
         * @param homographyMapper specifies the homographyMapper to use.
         * @param targetGroundOffset specifies ground offset of the target, zero if target is on the ground.
         * @return calculated target pose.
         */
        protected TrcPose2D getTargetPoseByHomography(TrcHomographyMapper homographyMapper, double targetGroundOffset)
        {
            if (pixelRect == null)
            {
                pixelRect = getPixelRect();
            }

            if (pixelRect != null)
            {
                Point bottomLeft = homographyMapper.mapPoint(new Point(pixelRect.x, pixelRect.y + pixelRect.height));
                Point bottomRight = homographyMapper.mapPoint(
                    new Point(pixelRect.x + pixelRect.width, pixelRect.y + pixelRect.height));
                // Assuming mid-bottom edge of rect is touching the ground.
                double xDistanceFromCamera = (bottomLeft.x + bottomRight.x)/2.0;
                double yDistanceFromCamera = (bottomLeft.y + bottomRight.y)/2.0;
                double bearingRad = Math.atan2(xDistanceFromCamera, yDistanceFromCamera);
                double bearingDeg = Math.toDegrees(bearingRad);
                targetDistance = TrcUtil.magnitude(xDistanceFromCamera, yDistanceFromCamera);
                if (targetGroundOffset > 0.0)
                {
                    // If target is elevated off the ground, the target distance would be further than it actually is.
                    // Therefore, we need to calculate the distance adjustment to be subtracted from the Homography
                    // reported distance. Imagine the camera is the sun casting a shadow on the target to the ground.
                    // The distance betwen the shadow and the target's ground location is the distance adjustment.
                    //
                    //  cameraHeight / homographyDistance = targetGroundOffset / adjustment
                    //  adjustment = targetGroundOffset * homographyDistance / cameraHeight
                    double adjustment =
                        targetGroundOffset*targetDistance/cameraInfo.camPose.z;
                    xDistanceFromCamera -= adjustment * Math.sin(bearingRad);
                    yDistanceFromCamera -= adjustment * Math.cos(bearingRad);
                    targetDistance -= adjustment;
                }
                // Don't have enough info to determine pitch and roll.
                targetPose2d = new TrcPose2D(xDistanceFromCamera, yDistanceFromCamera, bearingDeg);
                targetWidth = TrcUtil.magnitude(bottomRight.x - bottomLeft.x, bottomRight.y - bottomLeft.y);
            }

            return targetPose2d;
        }   //getTargetPoseByHomography

        /**
         * This method calculates the detected target pose by applying geometry on target pixel position and
         * camera FOVs.
         *
         * @param targetGroundOffset specifies the ground offset of the detected target.
         * @return calculated target pose.
         */
        protected TrcPose2D getTargetPoseByPixelPosition(double targetGroundOffset)
        {
            if (pixelRect == null)
            {
                getPixelRect();
            }

            if (pixelRect != null)
            {
                // Other pipelines only have 2D info (less accurate and potentially sensitive to error).
                // This method is very inaccurate when the target is at about the same height as the camera.
                // Any error in the camPitch angle will be amplified. It can also potentially give a divide-by-zero
                // error if the object is at the exact same height as the camera (i.e. camPitch+targetElevator==0).
                double camPitchRad = Math.toRadians(cameraInfo.camPose.pitch);
                double targetXPixel = pixelRect.x + pixelRect.width/2.0 - cameraInfo.lensInfo.cx;
                double targetYPixel = cameraInfo.lensInfo.cy - (pixelRect.y + pixelRect.height/2.0);
                double targetBearingRad   = Math.atan(targetXPixel / cameraInfo.lensInfo.fx);
                double targetElevationRad = Math.atan(targetYPixel / cameraInfo.lensInfo.fy);
                double targetBearingDeg = Math.toDegrees(targetBearingRad);
                double targetPitchFromGroundRad = camPitchRad + targetElevationRad;

                targetDistance = Math.abs(targetPitchFromGroundRad) < 1e-4?
                    Double.MAX_VALUE: (targetGroundOffset - cameraInfo.camPose.z)/Math.tan(targetPitchFromGroundRad);
                targetPose2d = new TrcPose2D(
                    targetDistance * Math.sin(targetBearingRad),
                    targetDistance * Math.cos(targetBearingRad),
                    targetBearingDeg);
                TrcDbgTrace.globalTraceDebug(
                    "TargetInfo",
                    "groundOffset=%.1f, cameraZ=%.1f, camPitch=%.1f, targetElevation=%.1f, targetDepth=%.1f, " +
                        "targetBearing=%.1f, targetPose2d=%s",
                    targetGroundOffset, cameraInfo.camPose.z, cameraInfo.camPose.pitch,
                    Math.toDegrees(targetElevationRad), targetDistance, targetBearingDeg, targetPose2d);
            }

            return targetPose2d;
        }   //getTargetPoseByPixelPosition

        /**
         * This method transforms a target pose in camera space into the main robot frame of reference.
         *
         * @param targetPoseCameraSpace specifies the 3D target pose in Camera Space.
         * @param cameraPose specifies the physical location and mounting orientation of the camera relative to the
         *        robot center.
         * @return target position in 2D robot space (Y forward, X right, heading CW from the positive Y axis).
         */
        public static TrcPose3D transformCameraSpaceToRobotSpace(TrcPose3D targetPoseCameraSpace, TrcPose3D cameraPose)
        {
            // Combine the target's relative camera-space pose onto the camera's physical mounting pose.
            // This rotates the target vector into global space and compounds the 3D orientations properly.
            return cameraPose.addRelativePose(targetPoseCameraSpace);
        }   //transformCameraSpaceToRobotSpace

        public static TrcPose2D project3dTo2dSpace(TrcPose3D target3dPose)
        {
            // Project components into TrcLib 2D space (Y forward, X right, heading CW from the Y-axis)
            // Using Math.atan2(x, y) establishes a 0-heading along the positive Y-axis, increasing CW toward positive X.
            return new TrcPose2D(
                target3dPose.x, target3dPose.y, Math.toDegrees(Math.atan2(target3dPose.x, target3dPose.y)));
        }   //project3dTo2dSpace
    }   //class TargetInfo

}   //class TrcVision
