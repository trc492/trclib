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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib.subsystem;

import java.util.ArrayList;
import java.util.HashMap;

public class TrcShootParamTable
{
    public static class Params
    {
        public final String loc;
        public final double distance;
        public final double shooter1Velocity;
        public final double shooter2Velocity;
        public final double tiltAngle;

        public Params(
            String loc, double distance, double shooter1Velocity, double shooter2Velocity, double tiltAngle)
        {
            this.loc = loc;
            this.distance = distance;
            this.shooter1Velocity = shooter1Velocity;
            this.shooter2Velocity = shooter2Velocity;
            this.tiltAngle = tiltAngle;
        }   //Params

        @Override
        public String toString()
        {
            return "{loc=" + loc +
                   ", distance=" + distance +
                   ", shooter1Vel=" + shooter1Velocity +
                   ", shooter2Vel=" + shooter2Velocity +
                   ", tiltAngle=" + tiltAngle + "}";
        }   //toString

    }   //class Params

    private final ArrayList<Params> paramTable;
    private final HashMap<String, Params> paramMap;

    /**
     * Constructor: Create an instance of the object.
     */
    public TrcShootParamTable()
    {
        paramTable = new ArrayList<>();
        paramMap = new HashMap<>();
    }   //TrcShootParamTable

    /**
     * This method adds an entry to the ShootParamTable sorted by distance.
     *
     * @param loc specifies the shoot location for the entry.
     * @param distance specifies the target distance.
     * @param shooter1Vel specifies the shooter 1 velocity in RPS.
     * @param shooter2Vel specifies the shooter 1 velocity in RPS.
     * @param tiltAngle specifies the tilt angle in degrees.
     *
     * @return this instance object.
     */
    public TrcShootParamTable add(
        String loc, double distance, double shooter1Vel, double shooter2Vel, double tiltAngle)
    {
        Params newEntry = new Params(loc, distance, shooter1Vel, shooter2Vel, tiltAngle);
        int insertPoint = paramTable.size();

        for (int i = 0; i < paramTable.size(); i++)
        {
            Params entry = paramTable.get(i);
            if (distance == entry.distance)
            {
                throw new RuntimeException("An entry with the same distance already exist.");
            }
            else if (distance < entry.distance)
            {
                insertPoint = i;
                break;
            }
        }

        paramTable.add(insertPoint, newEntry);
        paramMap.put(newEntry.loc, newEntry);
        return this;
    }   //add

    /**
     * This method returns the Shoot Param entry that matches the given name.
     *
     * @param loc specifies the shoot location for the entry.
     * @return shoot param entry that matches the shoot location, null if not found.
     */
    public Params get(String loc)
    {
        return paramMap.get(loc);
    }   //get

    /**
     * This method returns the Shooter velocity and tilt angle from the table entry that matches the given name.
     *
     * @param loc specifies the shoot location for the entry.
     * @return shooter velocity and tilt angle in an array, null if not found.
     */
    public double[] getShootParams(String loc)
    {
        double[] shootParams = null;
        Params params = paramMap.get(loc);

        if (params != null)
        {
            shootParams = new double[3];
            shootParams[0] = params.shooter1Velocity;
            shootParams[1] = params.shooter2Velocity;
            shootParams[2] = params.tiltAngle;
        }

        return shootParams;
    }   //getShootParams

    /**
     * This method returns the Shoot Param entry with the given distance. If there is no exact match, it will return
     * an entry that linearly interpolates between two entries in the table.
     *
     * @param distance specifies the distance to lookup in the table.
     * @param interpolateTiltAngle specifies true to interpolate Tilt Angle, false to interpolate Shooter Velocity.
     * @return shoot param entry that matches the distance. If no exact match, an interpolated entry is returned.
     *         If the distance is out of range of the table entries, an extrapolated entry is returned.
     */
    public Params get(double distance, boolean interpolateTiltAngle)
    {
        Params foundEntry = null;

        if (paramTable.size() < 2)
        {
            throw new RuntimeException("ShootParamTable must have at least 2 entries.");
        }

        Params firstEntry = paramTable.get(0);
        Params lastEntry = paramTable.get(paramTable.size() - 1);

        if (distance <= firstEntry.distance)
        {
            // The provided distance is below the table range, extropolate.
            Params nextEntry = paramTable.get(1);
            if (interpolateTiltAngle)
            {
                foundEntry = new Params(
                    "Extrapolated", distance,
                    firstEntry.shooter1Velocity,
                    firstEntry.shooter2Velocity,
                    extrapolateValue(
                        distance, firstEntry.distance, nextEntry.distance,
                        firstEntry.tiltAngle, nextEntry.tiltAngle));
            }
            else
            {
                foundEntry = new Params(
                    "Extrapolated", distance,
                    extrapolateValue(
                        distance, firstEntry.distance, nextEntry.distance,
                        firstEntry.shooter1Velocity, nextEntry.shooter1Velocity),
                    extrapolateValue(
                        distance, firstEntry.distance, nextEntry.distance,
                        firstEntry.shooter2Velocity, nextEntry.shooter2Velocity),
                    firstEntry.tiltAngle);
            }
        }
        else if (distance > lastEntry.distance)
        {
            // The provided distance is above the table range, extropolate.
            Params prevEntry = paramTable.get(paramTable.size() - 2);
            if (interpolateTiltAngle)
            {
                foundEntry = new Params(
                    "Extrapolated", distance,
                    lastEntry.shooter1Velocity,
                    lastEntry.shooter2Velocity,
                    extrapolateValue(
                        distance, prevEntry.distance, lastEntry.distance,
                        prevEntry.tiltAngle, lastEntry.tiltAngle));
            }
            else
            {
                foundEntry = new Params(
                    "Extrapolated", distance,
                    extrapolateValue(
                        distance, prevEntry.distance, lastEntry.distance,
                        prevEntry.shooter1Velocity, lastEntry.shooter1Velocity),
                    extrapolateValue(
                        distance, prevEntry.distance, lastEntry.distance,
                        prevEntry.shooter2Velocity, lastEntry.shooter2Velocity),
                    lastEntry.tiltAngle);
            }
        }
        else
        {
            for (int i = 1; i < paramTable.size(); i++)
            {
                Params entry = paramTable.get(i);

                if (distance <= entry.distance)
                {
                    Params prevEntry = paramTable.get(i - 1);
                    if (interpolateTiltAngle)
                    {
                        foundEntry = new Params(
                            "Interpolated", distance,
                            entry.shooter1Velocity,
                            entry.shooter2Velocity,
                            interpolateValue(
                                distance, prevEntry.distance, entry.distance,
                                prevEntry.tiltAngle, entry.tiltAngle));
                    }
                    else
                    {
                        foundEntry = new Params(
                            "Interpolated", distance,
                            interpolateValue(
                                distance, prevEntry.distance, entry.distance,
                                prevEntry.shooter1Velocity, entry.shooter1Velocity),
                            interpolateValue(
                                distance, prevEntry.distance, entry.distance,
                                prevEntry.shooter2Velocity, entry.shooter2Velocity),
                            entry.tiltAngle);
                    }
                    break;
                }
            }
        }

        return foundEntry;
    }   //get

    /**
     * This method interpolates the value with the given distance and two points of the neighboring segment in the
     * table.
     *
     * @param distance specifies the target distance.
     * @param distance1 specifies the distance of the lower entry of the neighboring segment.
     * @param distance2 specifies the distance of the upper entry of the neighboring segment.
     * @param value1 specifies the value of the lower entry of the neighboring segment (can be tilt angle or shooter
     *        velocity);
     * @param value2 specifies the value of the upper entry of the neighboring segment (can be tilt angle or shooter
     *        velocity);
     * @return interpolated value.
     */
    private double interpolateValue(double distance, double distance1, double distance2, double value1, double value2)
    {
        // TODO: interpolation should NOT be linear. It should have a tangent relationship with distance.
        double w = (distance - distance1) / (distance2 - distance1);
        return (1 - w) * value1 + w * value2;
    }   //interpolateValue

    /**
     * This method extrapolates the value with the given distance and the two points of the neighboring segment in the
     * table.
     *
     * @param distance specifies the target distance.
     * @param distance1 specifies the distance of the lower entry of the neighboring segment.
     * @param distance2 specifies the distance of the upper entry of the neighboring segment.
     * @param value1 specifies the value of the lower entry of the neighboring segment (can be tilt angle or shooter
     *        velocity);
     * @param value2 specifies the value of the upper entry of the neighboring segment (can be tilt angle or shooter
     *        velocity);
     * @return extrapolated value.
     */
    private double extrapolateValue(double distance, double distance1, double distance2, double value1, double value2)
    {
        // TODO: extrapolation should NOT be linear. It should have a tangent relationship with distance.
        double deltaValue = value2 - value1;
        double deltaDistance = distance2 - distance1;
        double m = deltaValue / deltaDistance;
        double b = value1 - m*distance1;
        double value = m*distance + b;

        if (value < 0.0)
        {
            // If extrapolated value is negative, just return the value of one of the two entries whoever is
            // closer to the given distance.
            double d1 = Math.abs(distance - distance1);
            double d2 = Math.abs(distance - distance2);
            value = d1 < d2? value1: value2;
        }

        return value;
    }   //extrapolateValue

}   //class TrcShootParamTable
