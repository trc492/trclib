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

import trclib.pathdrive.TrcPose3D;

/**
 * This class contains platform independent Vision information.
 */
public class TrcVision
{
    /**
     * This class contains camera information.
     */
    public static class CameraInfo
    {
        public String camName = null;
        public int camImageWidth = 0, camImageHeight = 0;
        public Double camHFov = null, camVFov = null;
        public TrcOpenCvDetector.LensInfo lensInfo = null;
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
        public CameraInfo setLensProperties(TrcOpenCvDetector.LensInfo lensInfo)
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
                new TrcOpenCvDetector.LensInfo().setLensProperties(fx, fy, cx, cy)
                                                .setDistortionCoefficents(distCoeffs));
            return this;
        }   //setLensProperties

        /**
         * This method sets the camera location relative to robot center on the ground.
         *
         * @param xOffset specifies the X offset from robot center (positive right).
         * @param yOffset specifies the Y offset from robot center (positive forward).
         * @param zOffset specifies the Z offset from the ground (positive up).
         * @param yaw specifies yaw angle from robot forward (positive clockwise).
         * @param pitch specifies pitch angle from horizontal (positive up).
         * @param roll specifies roll angle from vertical (positive left wing up).
         * @return this object for chaining.
         */
        public CameraInfo setCameraPose(
            double xOffset, double yOffset, double zOffset, double yaw, double pitch, double roll)
        {
            this.camPose = new TrcPose3D(xOffset, yOffset, zOffset, yaw, pitch, roll);
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

}   //class TrcVision
