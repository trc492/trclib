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
import java.util.Arrays;
import java.util.HashMap;

public class TrcShootParams
{
    public static class Region
    {
        public final double tiltAngle;
        public final double[][] polynomialCoeffs;

        public Region(double tiltAngle, double[][] polynomialCoeffs)
        {
            this.tiltAngle = tiltAngle;
            this.polynomialCoeffs = polynomialCoeffs;
        }   //Region

        @Override
        public String toString()
        {
            return "{tiltAngle=" + tiltAngle + ", coeffs=" + Arrays.deepToString(polynomialCoeffs) + "}";
        }   //toString
    }   //class Region

    public static class Entry
    {
        public final String name;
        public final double distance;
        public final Region region;
        public final double[] outputs;

        public Entry(String name, double distance, Region region, double[] outputs)
        {
            this.name = name;
            this.distance = distance;
            this.region = region;
            this.outputs = outputs;
        }   //Entry

        @Override
        public String toString()
        {
            return "{name=" + name +
                   ", distance=" + distance +
                   ", region=" + region +
                   ", outputs=" + Arrays.toString(outputs) + "}";
        }   //toString
    }   //class Entry

    private final ArrayList<Entry> shootParamsTable;
    private final HashMap<String, Entry> shootParamsMap;

    /**
     * Constructor: Create an instance of the object.
     */
    public TrcShootParams()
    {
        shootParamsTable = new ArrayList<>();
        shootParamsMap = new HashMap<>();
    }   //TrcShootParam

    /**
     * This method adds a new entry to the table by insertion sort.
     *
     * @param name specifies the name of the entry so it can be retrieved by name.
     * @param distance specifies the target distance.
     * @param region specifies the region the data point is in.
     * @param outputs specifies the output values.
     * @return table object for chaining.
     */
    public TrcShootParams addEntry(String name, double distance, Region region, double... outputs)
    {
        if (region.polynomialCoeffs.length != outputs.length)
        {
            throw new IllegalArgumentException(
                "The number of outputs must match the number of polynomial coefficient sets.");
        }

        Entry newEntry = new Entry(name, distance, region, outputs);
        int insertPoint = shootParamsTable.size();
        for (int i = 0; i < shootParamsTable.size(); i++)
        {
            Entry entry = shootParamsTable.get(i);

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
        shootParamsTable.add(insertPoint, newEntry);
        shootParamsMap.put(name, newEntry);

        return this;
    }   //addEntry

    /**
     * This method returns the Shoot Param entry that matches the given name.
     *
     * @param name specifies the entry name to look for.
     * @return shoot params entry that matches the name, null if not found.
     */
    public Entry get(String name)
    {
        return shootParamsMap.get(name);
    }   //get

    /**
     * This method returns the tilt angle and Shooter outputs from the table entry that matches the given name.
     *
     * @param name specifies the entry name to look for.
     * @return shooter velocity and tilt angle in an array, null if not found.
     */
    public double[] getShootParams(String name)
    {
        double[] shootParams = null;
        Entry entry = shootParamsMap.get(name);

        if (entry != null)
        {
            shootParams = new double[entry.outputs.length + 1];
            shootParams[0] = entry.region.tiltAngle;
            System.arraycopy(entry.outputs, 0, shootParams, 1, entry.outputs.length);
        }

        return shootParams;
    }   //getShootParams

    /**
     * This method returns the Shoot Params entry with the given distance. The entry is either calculated using
     * polynomial regression or linearly interpolated between two adjacent entries in the table.
     *
     * @param distance specifies the distance to lookup in the table.
     * @param useRegression specifies true to use polynomial regression, false to use table lookup with linear
     *        interpolation.
     * @return shoot params entry that is calculated from polynomial regression if specified, or a linear
     *         interpolated or extrapolated entry is returned.
     */
    public Entry get(double distance, boolean useRegression)
    {
        if (shootParamsTable.size() < 2)
        {
            throw new RuntimeException("ShootParamsTable must have at least 2 entries.");
        }

        Entry shootParams = null;
        Entry firstEntry = shootParamsTable.get(0);
        Entry lastEntry = shootParamsTable.get(shootParamsTable.size() - 1);
        if (distance <= firstEntry.distance)
        {
            shootParams = useRegression && firstEntry.region.polynomialCoeffs != null?
                calculateRegressionEntry(distance, firstEntry):
                // The provided distance is below the table range, extrapolate.
                calculateExtrapolatedEntry(distance, firstEntry, shootParamsTable.get(1));
        }
        else if (distance > lastEntry.distance)
        {
            shootParams = useRegression && lastEntry.region.polynomialCoeffs != null?
                calculateRegressionEntry(distance, lastEntry):
                // The provided distance is above the table range, extrapolate.
                calculateExtrapolatedEntry(distance, shootParamsTable.get(shootParamsTable.size() - 2), lastEntry);
        }
        else
        {
            for (int i = 1; i < shootParamsTable.size(); i++)
            {
                Entry entry = shootParamsTable.get(i);
                if (distance <= entry.distance)
                {
                    shootParams = useRegression && entry.region.polynomialCoeffs != null?
                        calculateRegressionEntry(distance, entry):
                        calculateInterpolatedEntry(distance, shootParamsTable.get(i - 1), entry);
                    break;
                }
            }
        }

        return shootParams;
    }   //get

    /**
     * This method creates an entry calculate using regression with the given distance and a lower neighboring entry
     * in the table.
     *
     * @param distance specifies the target distance.
     * @param entry specifies the neighboring entry.
     * @return regression entry.
     */
    public Entry calculateRegressionEntry(double distance, Entry entry)
    {
        double[] outputs = new double[entry.outputs.length];

        for (int i = 0; i < outputs.length; i++)
        {
            outputs[i] = entry.region.polynomialCoeffs[i][0];
            for (int j = 1; j < entry.region.polynomialCoeffs[i].length; j++)
            {
                outputs[i] += Math.pow(distance, j) * entry.region.polynomialCoeffs[i][j];
            }
        }

        return new Entry("RegressionEntry", distance, entry.region, outputs);
    }   //calculateRegressionEntry

    /**
     * This method creates an interpolated entry with the given distance and the two neighboring points in the table.
     *
     * @param distance specifies the target distance.
     * @param lowerEntry specifies the lower neighboring entry.
     * @param upperEntry specifies the upper neighboring entry.
     * @return interpolated entry.
     */
    private Entry calculateInterpolatedEntry(double distance, Entry lowerEntry, Entry upperEntry)
    {
        double[] outputs = new double[lowerEntry.outputs.length];
        double w = (distance - lowerEntry.distance) / (upperEntry.distance - lowerEntry.distance);

        for (int i = 0; i < outputs.length; i++)
        {
            outputs[i] = (1 - w) * lowerEntry.outputs[i] + w * upperEntry.outputs[i];
        }

        return new Entry("Interpolated", distance, lowerEntry.region, outputs);
    }   //calculateInterpolatedEntry

    /**
     * This method creates an extrapolated entry with the given distance and the two closest neighboring points in
     * the table.
     *
     * @param distance specifies the target distance.
     * @param lowerEntry specifies the lower neighboring entry.
     * @param upperEntry specifies the upper neighboring entry.
     * @return extrapolated entry.
     */
    private Entry calculateExtrapolatedEntry(double distance, Entry lowerEntry, Entry upperEntry)
    {
        double[] outputs = new double[lowerEntry.outputs.length];
        double deltaDistance = upperEntry.distance - lowerEntry.distance;

        for (int i = 0; i < outputs.length; i++)
        {
            double deltaOutput = upperEntry.outputs[i] - lowerEntry.outputs[i];
            double m = deltaOutput / deltaDistance;
            double b = lowerEntry.outputs[i] - m*lowerEntry.distance;
            outputs[i] = m * distance + b;
            if (outputs[i] < 0.0)
            {
                // If extrapolated value is negative, just return the value of one of the two entries whoever is
                // closer to the given distance.
                double d1 = Math.abs(distance - lowerEntry.distance);
                double d2 = Math.abs(distance - upperEntry.distance);
                outputs[i] = d1 < d2? lowerEntry.outputs[i]: upperEntry.outputs[i];
            }
        }

        return new Entry("Extrapolated", distance, upperEntry.region, outputs);
    }   //calculateExtrapolatedEntry

}   //class TrcShootParam
