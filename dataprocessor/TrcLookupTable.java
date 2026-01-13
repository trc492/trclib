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

package trclib.dataprocessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

public class TrcLookupTable
{
    public static class Region
    {
        public final double value;
        public final double[][] polynomialCoeffs;

        public Region(double value, double[][] polynomialCoeffs)
        {
            this.value = value;
            this.polynomialCoeffs = polynomialCoeffs;
        }   //Region

        @Override
        public String toString()
        {
            return "{value=" + value + ", coeffs=" + Arrays.deepToString(polynomialCoeffs) + "}";
        }   //toString
    }   //class Region

    public static class Entry
    {
        public final String name;
        public final double input;
        public final Region region;
        public final double[] outputs;

        public Entry(String name, double input, Region region, double[] outputs)
        {
            this.name = name;
            this.input = input;
            this.region = region;
            this.outputs = outputs;
        }   //Entry

        @Override
        public String toString()
        {
            return "{name=" + name +
                   ", input=" + input +
                   ", region=" + region +
                   ", outputs=" + Arrays.toString(outputs) + "}";
        }   //toString
    }   //class Entry

    private final ArrayList<Entry> lookupTable;
    private final HashMap<String, Entry> namedEntriesMap;

    /**
     * Constructor: Create an instance of the object.
     */
    public TrcLookupTable()
    {
        lookupTable = new ArrayList<>();
        namedEntriesMap = new HashMap<>();
    }   //TrcLookupTable

    /**
     * This method adds a new entry to the table by insertion sort.
     *
     * @param name specifies the name of the entry so it can be retrieved by name, can be null if not provided.
     * @param input specifies the lookup input.
     * @param region specifies the region the data point is in.
     * @param outputs specifies the output values.
     * @return table object for chaining.
     */
    public TrcLookupTable addEntry(String name, double input, Region region, double... outputs)
    {
        if (name != null && namedEntriesMap.get(name) != null)
        {
            throw new IllegalArgumentException("An entry with the same name already exist.");
        }

        if (region.polynomialCoeffs.length != outputs.length)
        {
            throw new IllegalArgumentException(
                "The number of outputs must match the number of polynomial coefficient sets.");
        }

        Entry newEntry = new Entry(name, input, region, outputs);
        int insertPoint = lookupTable.size();
        for (int i = 0; i < lookupTable.size(); i++)
        {
            Entry entry = lookupTable.get(i);

            if (input == entry.input)
            {
                throw new IllegalArgumentException("An entry with the same input already exist.");
            }
            else if (input < entry.input)
            {
                insertPoint = i;
                break;
            }
        }
        lookupTable.add(insertPoint, newEntry);
        if (name != null)
        {
            namedEntriesMap.put(name, newEntry);
        }

        return this;
    }   //addEntry

    /**
     * This method returns the Lookup Table entry that matches the given name.
     *
     * @param name specifies the entry name to look for.
     * @return Lookup Table entry that matches the name, null if not found.
     */
    public Entry get(String name)
    {
        return name != null? namedEntriesMap.get(name): null;
    }   //get

    /**
     * This method returns the calculated entry with the given input. The entry is either calculated using
     * polynomial regression or linearly interpolated between two adjacent entries in the table.
     *
     * @param input specifies the input value to lookup in the table.
     * @param useRegression specifies true to use polynomial regression, false to use table lookup with linear
     *        interpolation.
     * @return entry that is calculated from polynomial regression if specified, or a linear interpolated or
     *         extrapolated entry is returned.
     */
    public Entry get(double input, boolean useRegression)
    {
        if (lookupTable.size() < 2)
        {
            throw new RuntimeException("LookupTable must have at least 2 entries.");
        }

        Entry calculatedEntry = null;
        Entry firstEntry = lookupTable.get(0);
        Entry lastEntry = lookupTable.get(lookupTable.size() - 1);
        if (input <= firstEntry.input)
        {
            calculatedEntry = useRegression && firstEntry.region.polynomialCoeffs != null?
                calculateRegressionEntry(input, firstEntry):
                // The provided input is below the table range, extrapolate.
                calculateExtrapolatedEntry(input, firstEntry, lookupTable.get(1));
        }
        else if (input > lastEntry.input)
        {
            calculatedEntry = useRegression && lastEntry.region.polynomialCoeffs != null?
                calculateRegressionEntry(input, lastEntry):
                // The provided input is above the table range, extrapolate.
                calculateExtrapolatedEntry(input, lookupTable.get(lookupTable.size() - 2), lastEntry);
        }
        else
        {
            for (int i = 1; i < lookupTable.size(); i++)
            {
                Entry entry = lookupTable.get(i);
                if (input <= entry.input)
                {
                    calculatedEntry = useRegression && entry.region.polynomialCoeffs != null?
                        calculateRegressionEntry(input, entry):
                        calculateInterpolatedEntry(input, lookupTable.get(i - 1), entry);
                    break;
                }
            }
        }

        return calculatedEntry;
    }   //get

    /**
     * This method creates an entry calculated using regression with the given input and a lower neighboring entry
     * in the table.
     *
     * @param input specifies the input value.
     * @param entry specifies the neighboring entry.
     * @return regression entry.
     */
    public Entry calculateRegressionEntry(double input, Entry entry)
    {
        double[] outputs = new double[entry.outputs.length];

        for (int i = 0; i < outputs.length; i++)
        {
            outputs[i] = entry.region.polynomialCoeffs[i][0];
            for (int j = 1; j < entry.region.polynomialCoeffs[i].length; j++)
            {
                outputs[i] += Math.pow(input, j) * entry.region.polynomialCoeffs[i][j];
            }
        }

        return new Entry("Regression", input, entry.region, outputs);
    }   //calculateRegressionEntry

    /**
     * This method creates an interpolated entry with the given input and the two neighboring points in the table.
     *
     * @param input specifies the input value.
     * @param lowerEntry specifies the lower neighboring entry.
     * @param upperEntry specifies the upper neighboring entry.
     * @return interpolated entry.
     */
    private Entry calculateInterpolatedEntry(double input, Entry lowerEntry, Entry upperEntry)
    {
        double[] outputs = new double[lowerEntry.outputs.length];
        double w = (input - lowerEntry.input) / (upperEntry.input - lowerEntry.input);

        for (int i = 0; i < outputs.length; i++)
        {
            outputs[i] = (1 - w) * lowerEntry.outputs[i] + w * upperEntry.outputs[i];
        }

        return new Entry("Interpolated", input, lowerEntry.region, outputs);
    }   //calculateInterpolatedEntry

    /**
     * This method creates an extrapolated entry with the given input and the two closest neighboring points in
     * the table.
     *
     * @param input specifies the input value.
     * @param lowerEntry specifies the lower neighboring entry.
     * @param upperEntry specifies the upper neighboring entry.
     * @return extrapolated entry.
     */
    private Entry calculateExtrapolatedEntry(double input, Entry lowerEntry, Entry upperEntry)
    {
        double[] outputs = new double[lowerEntry.outputs.length];
        double deltaInput = upperEntry.input - lowerEntry.input;

        for (int i = 0; i < outputs.length; i++)
        {
            double deltaOutput = upperEntry.outputs[i] - lowerEntry.outputs[i];
            double m = deltaOutput / deltaInput;
            double b = lowerEntry.outputs[i] - m*lowerEntry.input;
            outputs[i] = m * input + b;
            if (outputs[i] < 0.0)
            {
                // If extrapolated value is negative, just return the value of one of the two entries whoever is
                // closer to the given input.
                double d1 = Math.abs(input - lowerEntry.input);
                double d2 = Math.abs(input - upperEntry.input);
                outputs[i] = d1 < d2? lowerEntry.outputs[i]: upperEntry.outputs[i];
            }
        }

        return new Entry("Extrapolated", input, upperEntry.region, outputs);
    }   //calculateExtrapolatedEntry

}   //class TrcLookupTable
