/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib.subsystem;

import trclib.motor.TrcServo;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;

/**
 * This class implements a platform independent Differential Servo Wrist Subsystem. A Differential Servo Wrist consists
 * of two servos controlling two degrees of freedom. The wrist can tilt as well as rotate. When the two servos turn
 * in the same direction on the mounted axis, the wrist tilts up and down. When the two servos turn in opposite
 * directions, the wrist rotates.
 */
public class TrcDifferentialServoWrist implements TrcExclusiveSubsystem
{
    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcServo servo1, servo2;

    private double tiltPower = 0.0;
    private double rotatePower = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param servo1 specifies the servo 1 object.
     * @param servo2 specifies the servo 2 object.
     */
    public TrcDifferentialServoWrist(String instanceName, TrcServo servo1, TrcServo servo2)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.servo1 = servo1;
        this.servo2 = servo2;
    }   //TrcDifferentialServoWrist

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
     * This method cancels previous wrist operation if applicable.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     */
    public void cancel(String owner)
    {
        servo1.cancel(owner);
        servo2.cancel(owner);
    }   //cancel

    /**
     * This method cancels previous wrist operation if applicable.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method sets the wrist tilting power.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the power of the wrist, can be zero if no delay.
     * @param power specifies how fast the wrist will tilt.
     */
    public void setTiltPower(String owner, double delay, double power)
    {
        tiltPower = power;
        servo1.setPower(owner, delay, power);
        servo2.setPower(owner, delay, power);
    }   //setTiltPower

    /**
     * This method sets the wrist tilting power.
     *
     * @param delay specifies the delay in seconds before setting the power of the wrist, can be zero if no delay.
     * @param power specifies how fast the wrist will tilt.
     */
    public void setTiltPower(double delay, double power)
    {
        setTiltPower(null, delay, power);
    }   //setTiltPower

    /**
     * This method sets the wrist tilting power.
     *
     * @param power specifies how fast the wrist will tilt.
     */
    public void setTiltPower(double power)
    {
        setTiltPower(null, 0.0, power);
    }   //setTiltPower

    /**
     * This method returns the last set tilt power value.
     *
     * @return last tilt power set to the wrist.
     */
    public double getTiltPower()
    {
        return tiltPower;
    }   //getTiltPower

    /**
     * This method sets the wrist rotating power.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the power of the wrist, can be zero if no delay.
     * @param power specifies how fast the wrist will rotate.
     */
    public void setRotatePower(String owner, double delay, double power)
    {
        rotatePower = power;
        servo1.setPower(owner, delay, power);
        servo2.setPower(owner, delay, -power);
    }   //setRotatePower

    /**
     * This method sets the wrist rotating power.
     *
     * @param delay specifies the delay in seconds before setting the power of the wrist, can be zero if no delay.
     * @param power specifies how fast the wrist will rotate.
     */
    public void setRotatePower(double delay, double power)
    {
        setRotatePower(null, delay, power);
    }   //setRotatePower

    /**
     * This method sets the wrist rotating power.
     *
     * @param power specifies how fast the wrist will rotate.
     */
    public void setRotatePower(double power)
    {
        setRotatePower(null, 0.0, power);
    }   //setRotatePower

    /**
     * This method returns the last set rotate power value.
     *
     * @return last rotate power set to the wrist.
     */
    public double getRotatePower()
    {
        return rotatePower;
    }   //getRotatePower

    /**
     * This method sets the tilt position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the tilt position of the wrist, can be zero if no
     *        delay.
     * @param position specifies the physical tilt position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setTiltPosition(String owner, double delay, double position, TrcEvent completionEvent, double timeout)
    {

    }   //setTiltPosition

    /**
     * This method sets the tilt position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param delay specifies the delay in seconds before setting the tilt position of the wrist, can be zero if no
     *        delay.
     * @param position specifies the physical tilt position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setTiltPosition(double delay, double position, TrcEvent completionEvent, double timeout)
    {
        setTiltPosition(null, delay, position, completionEvent, timeout);
    }   //setTiltPosition

    /**
     * This method sets the tilt position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param position specifies the physical tilt position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setTiltPosition(double position, TrcEvent completionEvent, double timeout)
    {
        setTiltPosition(null, 0.0, position, completionEvent, timeout);
    }   //setTiltPosition

    /**
     * This method sets the tilt position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param position specifies the physical tilt position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     */
    public void setTiltPosition(double position)
    {
        setTiltPosition(null, 0.0, position, null, 0.0);
    }   //setTiltPosition

    /**
     * This method returns the physical tilt position value of the wrist. Generally, servo do not provide real time
     * position feedback. Therefore, it will only return the position set by the last setPosition call.
     *
     * @return physical tilt position of the wrist, could be in degrees if setPhysicalPosRange is called to set the
     *         range in degrees.
     */
    public double getTiltPosition()
    {
        return (servo1.getPosition() + servo2.getPosition()) / 2.0;
    }   //getTiltPosition

    /**
     * This method sets the rotate position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the rotate position of the wrist, can be zero if no
     *        delay.
     * @param position specifies the physical rotate position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setRotatePosition(String owner, double delay, double position, TrcEvent completionEvent, double timeout)
    {

    }   //setRotatePosition

    /**
     * This method sets the rotate position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param delay specifies the delay in seconds before setting the rotate position of the wrist, can be zero if no
     *        delay.
     * @param position specifies the physical rotate position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setRotatePosition(double delay, double position, TrcEvent completionEvent, double timeout)
    {
        setRotatePosition(null, delay, position, completionEvent, timeout);
    }   //setRotatePosition

    /**
     * This method sets the rotate position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param position specifies the physical rotate position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setRotatePosition(double position, TrcEvent completionEvent, double timeout)
    {
        setRotatePosition(null, 0.0, position, completionEvent, timeout);
    }   //setRotatePosition

    /**
     * This method sets the rotate position of the wrist. By default, the servo maps its physical position the same as
     * its logical position [0.0, 1.0]. However, if setPhysicalPosRange was called, it could map a real world
     * physical range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event is given, it sets
     * event after the given amount of time has expired.
     * <p>
     * Servo operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree.
     * </p>
     *
     * @param position specifies the physical rotate position of the wrist. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     */
    public void setRotatePosition(double position)
    {
        setRotatePosition(null, 0.0, position, null, 0.0);
    }   //setRotatePosition

    /**
     * This method returns the physical rotate position value of the wrist. Generally, servo do not provide real time
     * position feedback. Therefore, it will only return the position set by the last setPosition call.
     *
     * @return physical rotate position of the wrist, could be in degrees if setPhysicalPosRange is called to set the
     *         range in degrees.
     */
    public double getRotatePosition()
    {
        return (servo1.getPosition() - servo2.getPosition()) / 2.0;
    }   //getRotatePosition

}   //class TrcDifferentialServoWrist
