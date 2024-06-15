/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import trclib.archive.TrcCardinalConverter;
import trclib.archive.TrcDbgTrace;
import trclib.archive.TrcFilter;
import trclib.archive.TrcOdometrySensor;
import trclib.dataprocessor.TrcDataIntegrator;
import trclib.timer.TrcElapsedTimer;

/**
 * This class implements a platform independent gyro. Typically, this class is extended by a platform dependent
 * gyro class. The platform dependent gyro class must implement the abstract methods required by this class. The
 * abstract methods allow this class to get raw data for each gyro axis. Depending on the options specified in the
 * constructor, this class creates an integrator or a CardinalConverter. The platform dependent gyro can specify
 * how many axes it supports by setting the HAS_AXIS options. If it does not provide heading data, it can set the
 * INTEGRATE option and let the built-in integrator handle it. Or if it provides a Cardinal heading instead of
 * Cartesian, it can set the CONVERT_TO_CARTESIAN option to enable the CardinalConverter to do the conversion.
 */
public abstract class TrcGyro extends TrcSensor<TrcGyro.DataType> implements TrcOdometrySensor
{
    public interface GyroData
    {
        /**
         * This method inverts the x-axis. This is useful if the orientation of the gyro x-axis is such that the data
         * goes the wrong direction.
         *
         * @param inverted specifies true to invert x-axis, false otherwise.
         */
        void setXInverted(boolean inverted);

        /**
         * This method inverts the y-axis. This is useful if the orientation of the gyro y-axis is such that the data
         * goes the wrong direction.
         *
         * @param inverted specifies true to invert y-axis, false otherwise.
         */
        void setYInverted(boolean inverted);

        /**
         * This method inverts the z-axis. This is useful if the orientation of the gyro z-axis is such that the data
         * goes the wrong direction.
         *
         * @param inverted specifies true to invert z-axis, false otherwise.
         */
        void setZInverted(boolean inverted);

        /**
         * This method returns the rotation rate on the x-axis.
         *
         * @return X rotation rate.
         */
        SensorData<Double> getXRotationRate();

        /**
         * This method returns the rotation rate on the y-axis.
         *
         * @return Y rotation rate.
         */
        SensorData<Double> getYRotationRate();

        /**
         * This method returns the rotation rate on the z-axis.
         *
         * @return Z rotation rate.
         */
        SensorData<Double> getZRotationRate();

        /**
         * This method returns the heading of the x-axis. If there is an integrator, we call the integrator to get
         * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
         * the platform dependent gyro to get the raw heading value.
         *
         * @return X heading.
         */
        SensorData<Double> getXHeading();

        /**
         * This method returns the heading of the y-axis. If there is an integrator, we call the integrator to get
         * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
         * the platform dependent gyro to get the raw heading value.
         *
         * @return Y heading.
         */
        SensorData<Double> getYHeading();

        /**
         * This method returns the heading of the z-axis. If there is an integrator, we call the integrator to get
         * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
         * the platform dependent gyro to get the raw heading value.
         *
         * @return Z heading.
         */
        SensorData<Double> getZHeading();

        /**
         * This method resets the integrator on the x-axis.
         */
        void resetXIntegrator();

        /**
         * This method resets the integrator on the y-axis.
         */
        void resetYIntegrator();

        /**
         * This method resets the integrator on the z-axis.
         */
        void resetZIntegrator();

    }   //interface GyroData
    //
    // Gyro data types.
    //
    public enum DataType
    {
        ROTATION_RATE,
        HEADING
    }   //enum DataType

    /**
     * This abstract method returns the raw data with the specified type of the x-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data with the specified type of the x-axis.
     */
    public abstract SensorData<Double> getRawXData(DataType dataType);

    /**
     * This abstract method returns the raw data with the specified type of the y-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data with the specified type of the y-axis.
     */
    public abstract SensorData<Double> getRawYData(DataType dataType);

    /**
     * This abstract method returns the raw data with the specified type of the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data with the specified type of the z-axis.
     */
    public abstract SensorData<Double> getRawZData(DataType dataType);

    //
    // Gyro options.
    //
    public static final int GYRO_HAS_X_AXIS             = (1 << 0);
    public static final int GYRO_HAS_Y_AXIS             = (1 << 1);
    public static final int GYRO_HAS_Z_AXIS             = (1 << 2);
    public static final int GYRO_INTEGRATE              = (1 << 3);
    public static final int GYRO_CONVERT_TO_CARTESIAN   = (1 << 4);

    private final Odometry odometry;
    private TrcDataIntegrator<DataType> integrator = null;
    private TrcCardinalConverter<DataType> cardinalConverter = null;
    private int xIndex = -1;
    private int yIndex = -1;
    private int zIndex = -1;
    private TrcElapsedTimer getDataElapsedTimer = null;
    private double zZeroPos = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes of the gyro.
     * @param options specifies the gyro options. Multiple options can be OR'd together.
     *                GYRO_HAS_X_AXIS - supports x-axis.
     *                GYRO_HAS_Y_AXIS - supports y-axis.
     *                GYRO_HAS_Z_AXIS - supports z-axis.
     *                GYRO_INTEGRATE - do integration on all axes to get headings.
     *                GYRO_CONVERT_TO_CARTESIAN - converts the Cardinal heading to Cartesian heading.
     * @param filters specifies an array of filter objects one for each supported axis. It is assumed that the order
     *                of the filters in the array is x, y and then z. If an axis is specified in the options but no
     *                filter will be used on that axis, the corresponding element in the array should be set to null.
     *                If no filter is used at all, filters can be set to null.
     */
    public TrcGyro(String instanceName, int numAxes, int options, TrcFilter[] filters)
    {
        super(instanceName, numAxes, filters);
        odometry = new Odometry(this);
        //
        // Count the number of axes and set up the indices for each axis.
        //
        int axisCount = 0;
        if ((options & GYRO_HAS_X_AXIS) != 0)
        {
            xIndex = axisCount;
            axisCount++;
        }

        if ((options & GYRO_HAS_Y_AXIS) != 0)
        {
            yIndex = axisCount;
            axisCount++;
        }

        if ((options & GYRO_HAS_Z_AXIS) != 0)
        {
            zIndex = axisCount;
            axisCount++;
        }

        if (axisCount != numAxes)
        {
            throw new IllegalArgumentException("numAxes doesn't match the number of axes in options");
        }

        //
        // Integration of rate and converting to Cartesian heading are mutually exclusive. If we are doing software
        // integration, the resulting heading is already Cartesian. If we need to convert to Cartesian heading, the
        // heading is from the physical sensor and not from the integrator.
        //
        if ((options & GYRO_INTEGRATE) != 0 && (options & GYRO_CONVERT_TO_CARTESIAN) != 0)
        {
            throw new IllegalArgumentException("Options Integrator and CardinalConverter cannot coexist.");
        }
        //
        // Create the data integrator.
        //
        if ((options & GYRO_INTEGRATE) != 0)
        {
            integrator = new TrcDataIntegrator<>(instanceName, this, DataType.ROTATION_RATE, false);
        }

        //
        // Create the data CardinalConverter.
        //
        if ((options & GYRO_CONVERT_TO_CARTESIAN) != 0)
        {
            cardinalConverter = new TrcCardinalConverter<>(instanceName, this, DataType.HEADING);
        }
    }   //TrcGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes of the gyro.
     * @param options specifies the gyro options. Multiple options can be OR'd together.
     *                GYRO_HAS_X_AXIS - supports x-axis.
     *                GYRO_HAS_Y_AXIS - supports y-axis.
     *                GYRO_HAS_Z_AXIS - supports z-axis.
     *                GYRO_INTEGRATE - do integration on all axes to get headings.
     *                GYRO_CONVERT_TO_CARTESIAN - converts the Cardinal heading to Cartesian heading.
     */
    public TrcGyro(String instanceName, int numAxes, int options)
    {
        this(instanceName, numAxes, options, null);
    }   //TrcGyro

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
     * This method enables/disables the processing of gyro data. It is not automatically enabled when the TrcGyro
     * object is created. You need to explicitly enable the it before data processing will start. As part of enabling
     * the gyro, calibrate() is also called. calibrate() may be overridden by the platform dependent gyro if it is
     * capable of doing its own. Otherwise, calibrate will call the built-in calibrator to do the calibration.
     * Enabling/disabling data processing for the gyro involves enabling/disabling the Integrator and the
     * CardinalConverter if they exist.
     *
     * @param enabled specifies true if enabling, false otherwise.
     */
    public void setEnabled(boolean enabled)
    {
        //
        // Enable/disable Integrator.
        //
        if (integrator != null)
        {
            integrator.setEnabled(enabled);
        }

        //
        // Enable/disable CardinalConverter.
        //
        if (cardinalConverter != null)
        {
            cardinalConverter.setEnabled(enabled);
        }
    }   //setEnabled

    /**
     * This method enables/disables the elapsed timers for performance monitoring.
     *
     * @param enabled specifies true to enable elapsed timers, false to disable.
     */
    public void setElapsedTimerEnabled(boolean enabled)
    {
        if (enabled)
        {
            if (getDataElapsedTimer == null)
            {
                getDataElapsedTimer = new TrcElapsedTimer(instanceName + ".getGyroData", 2.0);
            }
        }
        else
        {
            getDataElapsedTimer = null;
        }
    }   //setElapsedTimerEnabled

    /**
     * This method prints the elapsed time info using the given tracer.
     *
     * @param tracer specifies the tracer to be used to print the info.
     */
    public void printElapsedTime(TrcDbgTrace tracer)
    {
        if (getDataElapsedTimer != null)
        {
            getDataElapsedTimer.printElapsedTime(tracer);
        }
    }   //printElapsedTime

    /**
     * This method inverts the x-axis. This is useful if the orientation of the gyro x-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert x-axis, false otherwise.
     */
    public void setXInverted(boolean inverted)
    {
        setInverted(xIndex, inverted);
    }   //setXInverted

    /**
     * This method inverts the y-axis. This is useful if the orientation of the gyro y-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert y-axis, false otherwise.
     */
    public void setYInverted(boolean inverted)
    {
        setInverted(yIndex, inverted);
    }   //setYInverted

    /**
     * This method inverts the z-axis. This is useful if the orientation of the gyro z-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert z-axis, false otherwise.
     */
    public void setZInverted(boolean inverted)
    {
        setInverted(zIndex, inverted);
    }   //setZInverted

    /**
     * This method sets the scale factor for the data of the x-axis.
     *
     * @param scale specifies the x scale factor.
     */
    public void setXScale(double scale)
    {
        setScale(xIndex, scale);
    }   //setXScale

    /**
     * This method sets the scale factor for the data of the y-axis.
     *
     * @param scale specifies the y scale factor.
     */
    public void setYScale(double scale)
    {
        setScale(yIndex, scale);
    }   //setYScale

    /**
     * This method sets the scale factor for the data of the z-axis.
     *
     * @param scale specifies the z scale factor.
     */
    public void setZScale(double scale)
    {
        setScale(zIndex, scale);
    }   //setZScale

    /**
     * This method sets the heading value range of the x-axis. The value range is used by the CardinalConverter to
     * convert the heading to Cartesian heading.
     *
     * @param valueRangeLow specifies the low value of the x-axis range.
     * @param valueRangeHigh specifies the high value of the x-axis range.
     */
    public void setXValueRange(double valueRangeLow, double valueRangeHigh)
    {
        if (cardinalConverter != null)
        {
            cardinalConverter.setCardinalRange(xIndex, valueRangeLow, valueRangeHigh);
        }
    }   //setXValueRange

    /**
     * This method sets the heading value range of the y-axis. The value range is used by the CardinalConverter to
     * convert the heading to Cartesian heading.
     *
     * @param valueRangeLow specifies the low value of the y-axis range.
     * @param valueRangeHigh specifies the high value of the y-axis range.
     */
    public void setYValueRange(double valueRangeLow, double valueRangeHigh)
    {
        if (cardinalConverter != null)
        {
            cardinalConverter.setCardinalRange(yIndex, valueRangeLow, valueRangeHigh);
        }
    }   //setYValueRange

    /**
     * This method sets the heading value range of the z-axis. The value range is used by the CardinalConverter to
     * convert the heading to Cartesian heading.
     *
     * @param valueRangeLow specifies the low value of the z-axis range.
     * @param valueRangeHigh specifies the high value of the z-axis range.
     */
    public void setZValueRange(double valueRangeLow, double valueRangeHigh)
    {
        if (cardinalConverter != null)
        {
            cardinalConverter.setCardinalRange(zIndex, valueRangeLow, valueRangeHigh);
        }
    }   //setZValueRange

    /**
     * This method resets the CardinalConverter on the x-axis.
     */
    public void resetXCardinalConverter()
    {
        if (cardinalConverter != null)
        {
            cardinalConverter.reset(xIndex);
        }
    }   //resetXCardinalConverter

    /**
     * This method resets the CardinalConverter on the y-axis.
     */
    public void resetYCardinalConverter()
    {
        if (cardinalConverter != null)
        {
            cardinalConverter.reset(yIndex);
        }
    }   //resetYCardinalConverter

    /**
     * This method resets the CardinalConverter on the z-axis.
     */
    public void resetZCardinalConverter()
    {
        if (cardinalConverter != null)
        {
            cardinalConverter.reset(zIndex);
        }
    }   //resetZCardinalConverter

    /**
     * This method returns the rotation rate on the x-axis.
     *
     * @return X rotation rate.
     */
    public SensorData<Double> getXRotationRate()
    {
        return getProcessedData(xIndex, DataType.ROTATION_RATE);
    }   //getXRotationRate

    /**
     * This method returns the rotation rate on the y-axis.
     *
     * @return Y rotation rate.
     */
    public SensorData<Double> getYRotationRate()
    {
        return getProcessedData(yIndex, DataType.ROTATION_RATE);
    }   //getYRotationRate

    /**
     * This method returns the rotation rate on the z-axis.
     *
     * @return Z rotation rate.
     */
    public SensorData<Double> getZRotationRate()
    {
        return getProcessedData(zIndex, DataType.ROTATION_RATE);
    }   //getZRotationRate

    /**
     * This method returns the heading of the x-axis. If there is an integrator, we call the integrator to get the
     * heading. Else if we have a CardinalConverter, we call it to get the heading else we call the platform dependent
     * gyro to get the raw heading value.
     *
     * @return X heading.
     */
    public SensorData<Double> getXHeading()
    {
        SensorData<Double> data;

        if (integrator != null)
        {
            data = integrator.getIntegratedData(xIndex);
        }
        else if (cardinalConverter != null)
        {
            data = cardinalConverter.getCartesianData(xIndex);
        }
        else
        {
            data = getRawXData(DataType.HEADING);
        }

        return data;
    }   //getXHeading

    /**
     * This method returns the heading of the y-axis. If there is an integrator, we call the integrator to get the
     * heading. Else if we have a CardinalConverter, we call it to get the heading else we call the platform dependent
     * gyro to get the raw heading value.
     *
     * @return Y heading.
     */
    public SensorData<Double> getYHeading()
    {
        SensorData<Double> data;

        if (integrator != null)
        {
            data = integrator.getIntegratedData(yIndex);
        }
        else if (cardinalConverter != null)
        {
            data = cardinalConverter.getCartesianData(yIndex);
        }
        else
        {
            data = getRawYData(DataType.HEADING);
        }

        return data;
    }   //getYHeading

    /**
     * This method returns the heading of the z-axis. If there is an integrator, we call the integrator to get the
     * heading. Else if we have a CardinalConverter, we call it to get the heading else we call the platform dependent
     * gyro to get the raw heading value.
     *
     * @return Z heading.
     */
    public SensorData<Double> getZHeading()
    {
        SensorData<Double> data;

        if (integrator != null)
        {
            data = integrator.getIntegratedData(zIndex);
        }
        else if (cardinalConverter != null)
        {
            data = cardinalConverter.getCartesianData(zIndex);
        }
        else
        {
            data = getRawZData(DataType.HEADING);
        }

        return data;
    }   //getZHeading

    //
    // The following methods can be overridden by a platform dependent gyro class.
    //

    /**
     * This method resets the integrator on the x-axis.
     */
    public void resetXIntegrator()
    {
        if (integrator != null)
        {
            integrator.reset(xIndex);
        }
    }   //resetXIntegrator

    /**
     * This method resets the integrator on the y-axis.
     */
    public void resetYIntegrator()
    {
        if (integrator != null)
        {
            integrator.reset(yIndex);
        }
    }   //resetYIntegrator

    /**
     * This method resets the integrator on the z-axis.
     */
    public void resetZIntegrator()
    {
        if (integrator != null)
        {
            integrator.reset(zIndex);
        }
    }   //resetZIntegrator

    //
    // Implements TrcSensor abstract methods.
    //

    /**
     * This abstract method returns the raw sensor data for the specified axis and type.
     *
     * @param index specifies the axis index.
     * @param dataType specifies the data type.
     * @return raw data for the specified axis and type.
     */
    @Override
    public SensorData<Double> getRawData(int index, DataType dataType)
    {
        SensorData<Double> data = null;

        if (getDataElapsedTimer != null) getDataElapsedTimer.recordStartTime();
        if (index == xIndex)
        {
            data = getRawXData(dataType);
        }
        else if (index == yIndex)
        {
            data = getRawYData(dataType);
        }
        else if (index == zIndex)
        {
            data = getRawZData(dataType);
        }
        if (getDataElapsedTimer != null) getDataElapsedTimer.recordEndTime();

        return data;
    }   //getRawData

    /**
     * This method reads the raw gyro Z data and updates the odometry with it.
     */
    private void updateOdometry()
    {
        TrcSensor.SensorData<Double> zHeading = getZHeading();

        odometry.prevTimestamp = odometry.currTimestamp;
        odometry.prevPos = odometry.currPos;
        odometry.currTimestamp = zHeading.timestamp;
        odometry.currPos = zHeading.value - zZeroPos;
        double timeDelta = odometry.currTimestamp - odometry.prevTimestamp;
        if (timeDelta != 0.0)
        {
            odometry.velocity = (odometry.currPos - odometry.prevPos) / timeDelta;
        }
    }   //updateOdometry

    //
    // Implements TrcOdometrySensor interface.
    //

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String getName()
    {
        return instanceName;
    }   //getName

    /**
     * This method resets the odometry data and sensor. TODO: need to support multiple axes.
     *
     * @param resetHardware specifies true to do a hardware reset, false to do a software reset. Hardware reset may
     *                      require some time to complete and will block this method from returning until finish.
     */
    @Override
    public void resetOdometry(boolean resetHardware)
    {
        synchronized (odometry)
        {
            if (resetHardware)
            {
                resetZIntegrator();
            }

            SensorData<Double> data = getProcessedData(zIndex, DataType.HEADING);
            zZeroPos = data.value;

            updateOdometry();
            odometry.prevTimestamp = odometry.currTimestamp;
            odometry.prevPos = odometry.currPos;
        }
    }   //resetOdometry

    /**
     * This method returns a copy of the odometry data of the specified axis. It must be a copy so it won't change while
     * the caller is accessing the data fields.
     *
     * @param axisIndex specifies the axis index if it is a multi-axes sensor, 0 if it is a single axis sensor.
     * @return a copy of the odometry data of the specified axis.
     */
    @Override
    public Odometry getOdometry(int axisIndex)
    {
        synchronized (odometry)
        {
            updateOdometry();
            return odometry.clone();
        }
    }   //getOdometry

}   //class TrcGyro
