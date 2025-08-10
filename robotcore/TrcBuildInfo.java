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

package trclib.robotcore;

import java.io.InputStream;
import java.util.Properties;

public class TrcBuildInfo
{
    private static final String moduleName = TrcBuildInfo.class.getSimpleName();
    private static final String buildInfoFileName = "buildInfo.properties";
    private static final Properties properties = new Properties();
    private static TrcBuildInfo buildInfo = null;
    public String buildHost;
    public String buildUser;
    public String buildTimestamp;
    public String buildBranch;

    /**
     * This method returns the build info including the host, user, timestamp and branch that built the code.
     * If this is the first time it is called, it will create the TrcBuildInfo object.
     *
     * @return build info.
     */
    public static TrcBuildInfo getBuildInfo()
    {
        if (buildInfo == null)
        {
            buildInfo = new TrcBuildInfo();
        }

        return buildInfo;
    }   //getBuildInfo

    /**
     * Constructor: Create an instance of the object and initialize the fields accordingly.
     */
    private TrcBuildInfo()
    {
        try (InputStream input = getClass().getClassLoader().getResourceAsStream(buildInfoFileName))
        {
            if (input != null)
            {
                properties.load(input);
                buildHost = properties.getProperty("build.host", "unknown");
                buildUser = properties.getProperty("build.user", "unknown");
                buildTimestamp = properties.getProperty("build.timestamp", "unknown");
                buildBranch = properties.getProperty("build.branch", "unknown");
            }
            else
            {
                TrcDbgTrace.globalTraceWarn(moduleName, "Failed to find " + buildInfoFileName);
            }
        }
        catch (Exception e)
        {
            TrcDbgTrace.globalTraceWarn(moduleName, "Failed to load resource: " + e.getMessage());
        }
    }   //TrcBuildInfo

    /**
     * This method returns the string form of the build info.
     *
     * @return build info string.
     */
    @Override
    public String toString()
    {
        return "Host=\"" + buildHost + "\"" +
               " User=\"" + buildUser + "\"" +
               " Timestamp=\"" + buildTimestamp + "\"" +
               " Branch=\"" + buildBranch + "\"";
    }   //toString

}   //class TrcBuildInfo
