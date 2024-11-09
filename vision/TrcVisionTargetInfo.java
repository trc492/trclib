/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 * Based on sample code by Robert Atkinson.
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

import java.util.Locale;

import trclib.pathdrive.TrcPose2D;
import trclib.dataprocessor.TrcUtil;

/**
 * This class calculates and stores the info for a vision detected target.
 */
public class TrcVisionTargetInfo<O extends TrcVisionTargetInfo.ObjectInfo>
{
    /**
     * This interface implements a method to get the rectangle of the detected object. This should be implemented by
     * a vision detector class.
     */
    public interface ObjectInfo
    {
        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        Rect getObjectRect();

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        double getObjectArea();

        /**
         * This method returns the projected 2D pose on the ground of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        TrcPose2D getObjectPose();

        /**
         * This method returns the objects real world width.
         *
         * @return object real world width, null if not supported.
         */
        Double getObjectWidth();

        /**
         * This method returns the objects real world depth.
         *
         * @return object real world depth, null if not supported.
         */
        Double getObjectDepth();

        /**
         * This method returns the rotated rect vertices of the detected object.
         *
         * @return rotated rect vertices.
         */
        Point[] getRotatedRectVertices();

    }   //interface ObjectInfo

    public O detectedObj;
    public Rect objRect;
    public double objArea;
    public TrcPose2D objPose;
    public Double objWidth;
    public Double objDepth;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param detectedObj specifies the detected object.
     * @param homographyMapper specifies the homography mapper, can be null if not provided in which case
     *        distanceFromCamera, targetWidth and horizontalAngle will not be determined.
     * @param objGroundOffset specifies the object ground offset above the floor, used by homography. Can be zero if
     *        homographyMapper is null.
     * @param cameraHeight specifies the height of the camera above the floor, used by homography. Can be zero if
     *        homographyMapper is null.
     */
    public TrcVisionTargetInfo(
        O detectedObj, TrcHomographyMapper homographyMapper, double objGroundOffset, double cameraHeight)
    {
        this.detectedObj = detectedObj;
        this.objRect = detectedObj.getObjectRect();
        this.objArea = detectedObj.getObjectArea();

        if (homographyMapper == null)
        {
            // Caller did not provide homography mapper, it means the caller is doing pose/width/depth calculation
            // itself.
            objPose = detectedObj.getObjectPose();
            objWidth = detectedObj.getObjectWidth();
            objDepth = detectedObj.getObjectDepth();
        }
        else
        {
            // Caller provided homography mapper, we will use it to calculate the detected object pose.
            Point topLeft = homographyMapper.mapPoint(new Point(objRect.x, objRect.y));
            Point topRight = homographyMapper.mapPoint(new Point(objRect.x + objRect.width, objRect.y));
            Point bottomLeft = homographyMapper.mapPoint(new Point(objRect.x, objRect.y + objRect.height));
            Point bottomRight = homographyMapper.mapPoint(new Point(objRect.x + objRect.width, objRect.y + objRect.height));
            double xDistanceFromCamera = (bottomLeft.x + bottomRight.x)/2.0;
            double yDistanceFromCamera = (bottomLeft.y + bottomRight.y)/2.0;
            double horiAngleRadian = Math.atan2(xDistanceFromCamera, yDistanceFromCamera);
            double horizontalAngle = Math.toDegrees(horiAngleRadian);
            if (objGroundOffset > 0.0)
            {
                // If object is elevated off the ground, the object distance would be further than it actually is.
                // Therefore, we need to calculate the distance adjustment to be subtracted from the Homography
                // reported distance. Imagine the camera is the sun casting a shadow on the object to the ground.
                // The shadow length is the distance adjustment.
                //
                //  cameraHeight / homographyDistance = objGroundOffset / adjustment
                //  adjustment = objGroundOffset * homographyDistance / cameraHeight
                double adjustment =
                    objGroundOffset*TrcUtil.magnitude(xDistanceFromCamera, yDistanceFromCamera)/cameraHeight;
                xDistanceFromCamera -= adjustment * Math.sin(horiAngleRadian);
                yDistanceFromCamera -= adjustment * Math.cos(horiAngleRadian);
            }
            // Don't have enough info to determine pitch and roll.
            objPose = new TrcPose2D(xDistanceFromCamera, yDistanceFromCamera, horizontalAngle);
            objWidth = bottomRight.x - bottomLeft.x;
            objDepth = ((topLeft.y + topRight.y) - (bottomLeft.y + bottomRight.y))/2.0;
        }
    }   //TrcVisionTargetInfo

    /**
     * Constructor: Create an instance of the object.
     *
     * @param detectedObj specifies the detected object.
     */
    public TrcVisionTargetInfo(O detectedObj)
    {
        this(detectedObj, null, 0.0, 0.0);
    }   //TrcVisionTargetInfo

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
            "pose=%s,rect=%s,bottomMidPoint(%d,%d),area=%.0f,width=%.1f,depth=%.1f",
            objPose, objRect, objRect != null? objRect.x + objRect.width/2: 0,
            objRect != null? objRect.y + objRect.height: 0, objArea, objWidth != null? objWidth: 0.0,
            objDepth != null? objDepth: 0.0);
    }   //toString

}   //class TrcVisionTargetInfo
