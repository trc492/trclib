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

package trclib.pathdrive;

import java.util.ArrayList;

import trclib.drivebase.TrcDriveBase;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class implements the auto-assist grid drive. It allows the driver to use the DPad to quickly navigate the
 * field maze in grid cell units in a square pattern accurately without the risk of running into obstacles at grid
 * cell intersections. This algorithm assumes we have accurate odometry. If we don't, all bets are off.
 */
public class TrcGridDrive
{
    private static final String moduleName = TrcGridDrive.class.getSimpleName();

    public final TrcDbgTrace tracer;
    private final TrcDriveBase driveBase;
    private final TrcPurePursuitDrive purePursuitDrive;
    private final double gridCellSize;
    private final double turnStartAdj;
    private final double turnEndAdj;
    private final TrcTaskMgr.TaskObject gridDriveTaskObj;
    private final TrcEvent callbackEvent;
    private final ArrayList<TrcPose2D> gridDriveQueue = new ArrayList<>();

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param purePursuitDrive specifies the pure pursuit drive object.
     * @param gridCellSize specifies the grid cell size in inches.
     * @param turnStartAdj specifies the distance adjustment for the previous segment endpoint before the turn.
     * @param turnEndAdj specifies the distance adjustment for the next segment startpoint after the turn.
     */
    public TrcGridDrive(
        TrcDriveBase driveBase, TrcPurePursuitDrive purePursuitDrive, double gridCellSize, double turnStartAdj,
        double turnEndAdj)
    {
        this.tracer = new TrcDbgTrace();
        this.driveBase = driveBase;
        this.purePursuitDrive = purePursuitDrive;
        this.gridCellSize = Math.abs(gridCellSize);
        this.turnStartAdj = Math.abs(turnStartAdj);
        this.turnEndAdj = Math.abs(turnEndAdj);
        gridDriveTaskObj = TrcTaskMgr.createTask("gridDriveTask", this::gridDriveTask);
        callbackEvent = new TrcEvent(moduleName + ".callbackEvent");
    }   //TrcGridDrive

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param purePursuitDrive specifies the pure pursuit drive object.
     * @param gridCellSize specifies the grid cell size in inches.
     */
    public TrcGridDrive(TrcDriveBase driveBase, TrcPurePursuitDrive purePursuitDrive, double gridCellSize)
    {
        this(driveBase, purePursuitDrive, gridCellSize, 0.0, 0.0);
    }   //TrcGridDrive

    /**
     * This method cancels Grid Drive if one is in progress.
     */
    public void cancel()
    {
        // Note: this is a cooperative multi-task, so don't need to be thread-safe.
        tracer.traceDebug(moduleName, "Canceling Grid Drive.");
        setTaskEnabled(false);
        gridDriveQueue.clear();
        purePursuitDrive.cancel();
        if (driveBase.hasOwnership(moduleName))
        {
            driveBase.releaseExclusiveAccess(moduleName);
        }
    }   //cancel

    /**
     * This method checks if Grid Drive is currently in progress.
     *
     * @return true if Grid Drive is active, false otherwise.
     */
    public boolean isGridDriveActive()
    {
        return gridDriveTaskObj.isRegistered();
    }   //isGridDriveActive

    /**
     * This method enables/disables the Grid Drive task.
     *
     * @param enabled specifies true to enable Grid Drive task, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        boolean taskActive = isGridDriveActive();

        if (!taskActive && enabled)
        {
            gridDriveTaskObj.registerTask(TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
            tracer.traceDebug(moduleName, "Enabling task.");
        }
        else if (taskActive && !enabled)
        {
            gridDriveTaskObj.unregisterTask();
            tracer.traceDebug(moduleName, "Disabling task.");
        }
    }   //setTaskEnabled

    /**
     * This method adjusts the grid cell position to the grid cell center.
     *
     * @param gridCellPos specifies the position in grid cell units.
     * @return adjusted position in grid cell units.
     */
    private double adjustToGridCellCenter(double gridCellPos)
    {
        return Math.signum(gridCellPos) * (((int) Math.abs(gridCellPos)) + 0.5);
    }   //adjustToGridCellCenter

    /**
     * This method adjusts the heading to the nearest 90-degree multiple.
     *
     * @param heading specifies the heading to be adjusted to the nearest 90-degree multiple.
     * @return adjusted heading in the range of [0.0, 360.0)
     */
    private double adjustGridCellHeading(double heading)
    {
        heading %= 360.0;
        if (heading < 0.0) heading += 360.0;
        return (((int) (heading / 90.0 + 0.5)) * 90.0) % 360.0;
    }   //adjustGridCellHeading

    /**
     * This method translates a real world pose to a grid cell pose.
     *
     * @param pose specifies the pose in real world units.
     * @return pose in the grid cell units.
     */
    public TrcPose2D poseToGridCell(TrcPose2D pose)
    {
        return new TrcPose2D(pose.x / gridCellSize, pose.y / gridCellSize, pose.angle);
    }   //poseToGridCell

    /**
     * This method translates a grid cell pose to a real world pose.
     *
     * @param gridCell specifies the grid cell pose in grid cell units.
     * @return pose in real world units.
     */
    public TrcPose2D gridCellToPose(TrcPose2D gridCell)
    {
        return new TrcPose2D(gridCell.x * gridCellSize, gridCell.y * gridCellSize, gridCell.angle);
    }   //gridCellToPose

    /**
     * This method takes a grid cell position that may not be at the center of the cell and adjusts it to the center
     * of the grid cell it's on.
     *
     * @param gridCell specifies the grid cell to adjust in the unit of cells.
     * @return adjusted gridCell in the unit of cells.
     */
    public TrcPose2D adjustGridCellCenter(TrcPose2D gridCell)
    {
        return new TrcPose2D(
            adjustToGridCellCenter(gridCell.x), adjustToGridCellCenter(gridCell.y),
            adjustGridCellHeading(gridCell.angle));
    }   //adjustGridCellCenter

    /**
     * This method adjusts the given pose to the nearest grid cell center pose.
     *
     * @param pose specifies the pose in real world units to be adjusted.
     * @return adjusted pose in real world units.
     */
    public TrcPose2D adjustPoseToGridCellCenter(TrcPose2D pose)
    {
        return gridCellToPose(adjustGridCellCenter(poseToGridCell(pose)));
    }   //adjustPoseToGridCellCenter

    /**
     * This method resets the drive base odometry to the nearest grid cell center. Odometry may get inaccurate for
     * various reasons such as odometry wheel slippage, sensor drifting etc. This situation can be corrected by
     * manually driving the robot to the center of a grid cell and call this method. It assumes the drift is within
     * the current grid cell. Therefore, it resets the odometry back to the nearest grid cell center.
     */
    public void resetGridCellCenter()
    {
        TrcPose2D robotPose = driveBase.getFieldPosition();
        TrcPose2D cellCenterPose = adjustPoseToGridCellCenter(robotPose);
        // Do not change the heading, use original robot pose heading. We are resetting just the position.
        cellCenterPose.angle = robotPose.angle;
        driveBase.setFieldPosition(cellCenterPose);
        tracer.traceDebug(
            moduleName,
            "robotPose=" + robotPose +
            ", cellCenterPose=" + cellCenterPose);
    }   //resetGridCellCenter

    /**
     * This method returns an intermediate grid cell so that travelling from the start cell to the end cell will
     * go through the intermediate cell such that the robot will only travel in the direction of north/south and
     * east/west but not diagonally and will be centering itself in the grid cells.
     *
     * @param startCell specifies the start cell.
     * @param endCell specifies the end cell.
     * @return intermediate cell if one is needed, null if end cell is already inline with the start cell.
     */
    private TrcPose2D getIntermediateGridCell(TrcPose2D startCell, TrcPose2D endCell)
    {
        TrcPose2D intermediateCell = null;

        if (startCell.x != endCell.x && startCell.y != endCell.y)
        {
            intermediateCell = startCell.clone();
            if (startCell.angle == 0.0 || startCell.angle == 180.0)
            {
                // Robot heading is north or south.
                intermediateCell.y = endCell.y;
            }
            else
            {
                // Robot heading is east or west.
                intermediateCell.x = endCell.x;
            }
        }

        return intermediateCell;
    }   //getIntermediateGridCell

    /**
     * This method adds a movement segment to the drive queue. Note: it is expected that only one argument will be
     * non-zero and the other one must be zero.
     *
     * @param xGridCells specifies the X movement in number of grid cells, positive for East, negative for West.
     * @param yGridCells specifies the Y movement in number of grid cells, positive for North, negative for South.
     */
    private void setRelativeGridTarget(int xGridCells, int yGridCells)
    {
        if (xGridCells != 0 && yGridCells == 0 || yGridCells != 0 && xGridCells == 0)
        {
            gridDriveQueue.add(new TrcPose2D(xGridCells, yGridCells, 0.0));
            tracer.traceDebug(
                moduleName,
                "CellUnits=" + xGridCells + "/" + yGridCells +
                " (QSize=" + gridDriveQueue.size() + ")");
            setTaskEnabled(true);
        }
    }   //setRelativeGridTarget

    /**
     * This method adds an X movement segment to the drive queue.
     *
     * @param gridCells specifies the X movement in number of grid cells, positive for East, negative for West.
     */
    public void setRelativeXGridTarget(int gridCells)
    {
        setRelativeGridTarget(gridCells, 0);
    }   //setRelativeXGridTarget

    /**
     * This method adds an Y movement segment to the drive queue.
     *
     * @param gridCells specifies the Y movement in number of grid cells, positive for North, negative for South.
     */
    public void setRelativeYGridTarget(int gridCells)
    {
        setRelativeGridTarget(0, gridCells);
    }   //setRelativeYGridTarget

    /**
     * This method generate a path from the current robot pose to the endpoint that snaps to the grid so it moves
     * only in square pattern, never diagonal.
     *
     * @param endPoint specifies the endpoint in real world units.
     */
    public void driveToEndPoint(TrcPose2D endPoint)
    {
        if (driveBase.acquireExclusiveAccess(moduleName))
        {
            TrcPose2D robotPose = driveBase.getFieldPosition();
            TrcPose2D startGridCell = adjustGridCellCenter(poseToGridCell(robotPose));
            TrcPose2D endGridCell = adjustGridCellCenter(poseToGridCell(endPoint));
            TrcPose2D intermediateGridCell = getIntermediateGridCell(startGridCell, endGridCell);
            TrcPathBuilder pathBuilder = new TrcPathBuilder(robotPose, false).append(gridCellToPose(startGridCell));

            if (intermediateGridCell != null)
            {
                pathBuilder.append(gridCellToPose(intermediateGridCell));
            }

            pathBuilder.append(gridCellToPose(endGridCell)).append(endPoint);
            TrcPath path = pathBuilder.toRelativeStartPath();
            tracer.traceDebug(moduleName, "EndPoint=" + endPoint + ", DrivePath=" + path);
            callbackEvent.setCallback(this::driveDone, null);
            purePursuitDrive.start(moduleName, path, callbackEvent, 0.0);
        }
    }   //driveToEndPoint

    /**
     * This method is called periodically to process the drive queue.
     *
     * @param taskType specifies the type of task being run. This may be useful for handling multiple task types.
     * @param runMode specifies the current competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void gridDriveTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        if (!purePursuitDrive.isActive())
        {
            if (gridDriveQueue.isEmpty())
            {
                // Nothing to do, cleanup and disable task.
                // The task will be enabled when something is added to the drive queue.
                cancel();
            }
            else
            {
                // Something in the drive queue, go process the queue.
                startGridDrive();
            }
        }
    }   //gridDriveTask

    /**
     * This method processes the gridDriveQueue to build a path for PurePursuitDrive to follow.
     */
    private void startGridDrive()
    {
        if (driveBase.acquireExclusiveAccess(moduleName))
        {
            TrcPose2D robotPose = driveBase.getFieldPosition();
            TrcPose2D startGridPose = adjustPoseToGridCellCenter(robotPose);
            TrcPose2D prevSegment = new TrcPose2D(0.0, 0.0, startGridPose.angle);
            // The first point of the path is center of the grid cell the robot is on.
            TrcPathBuilder pathBuilder = new TrcPathBuilder(robotPose, false).append(startGridPose);
            tracer.traceVerbose(
                moduleName,
                "##### robotPose=" + robotPose +
                ", startGridPose=" + startGridPose +
                ", QSize=" + gridDriveQueue.size());

            boolean deferSegment = false;
            while (gridDriveQueue.size() > 0)
            {
                TrcPose2D nextSegment = gridDriveQueue.get(0);

                tracer.traceVerbose(
                    moduleName,
                    "Adjacent Segments: prevSegment=" + prevSegment +
                    ", nextSegment=" + nextSegment);
                if (!willTurn(prevSegment, nextSegment))
                {
                    // Not turning, can coalesce the nextSegment to the prevSegment.
                    prevSegment.x += nextSegment.x;
                    prevSegment.y += nextSegment.y;
                    gridDriveQueue.remove(nextSegment);
                    tracer.traceVerbose(
                        moduleName,
                        "Coalesced compatible segments: prevSegment=" + prevSegment +
                        " (QSize=" + gridDriveQueue.size() + ")");
                    deferSegment = true;
                }
                else
                {
                    TrcPose2D prevEndPoint = prevSegment.clone();
                    TrcPose2D nextStartPoint = new TrcPose2D();
                    TrcPose2D nextSegmentPoint = nextSegment.clone();
                    double forwardAdj = turnStartAdj > 0.0? 1.0: 0.0;
                    // Turning, create an endpoint for the first segment and a startpoint for the next segment.
                    if (prevSegment.angle == 0.0)
                    {
                        // Heading is North.
                        prevEndPoint.y += turnStartAdj;
                        if (nextSegment.x > 0.0)
                        {
                            nextStartPoint.x = prevSegment.x + turnEndAdj;
                            nextStartPoint.y = prevSegment.y + forwardAdj;
                            nextStartPoint.angle = 90.0;
                        }
                        else
                        {
                            nextStartPoint.x = prevSegment.x - turnEndAdj;
                            nextStartPoint.y = prevSegment.y + forwardAdj;
                            nextStartPoint.angle = 270.0;
                        }
                        nextSegmentPoint.x += prevSegment.x;
                        nextSegmentPoint.y = nextStartPoint.y;
                    }
                    else if (prevSegment.angle == 180.0)
                    {
                        // Heading is South.
                        prevEndPoint.y -= turnStartAdj;
                        if (nextSegment.x > 0.0)
                        {
                            nextStartPoint.x = prevSegment.x + turnEndAdj;
                            nextStartPoint.y = prevSegment.y - forwardAdj;
                            nextStartPoint.angle = 90.0;
                        }
                        else
                        {
                            nextStartPoint.x = prevSegment.x - turnEndAdj;
                            nextStartPoint.y = prevSegment.y - forwardAdj;
                            nextStartPoint.angle = 270.0;
                        }
                        nextSegmentPoint.x += prevSegment.x;
                        nextSegmentPoint.y = nextStartPoint.y;
                    }
                    else if (prevSegment.angle == 90.0)
                    {
                        // Headihg is East.
                        prevEndPoint.x += turnStartAdj;
                        if (nextSegment.y > 0.0)
                        {
                            nextStartPoint.x = prevSegment.x + forwardAdj;
                            nextStartPoint.y = prevSegment.y + turnEndAdj;
                            nextStartPoint.angle = 0.0;
                        }
                        else
                        {
                            nextStartPoint.x = prevSegment.x + forwardAdj;
                            nextStartPoint.y = prevSegment.y - turnEndAdj;
                            nextStartPoint.angle = 180.0;
                        }
                        nextSegmentPoint.x = nextStartPoint.x;
                        nextSegmentPoint.y += prevSegment.y;
                    }
                    else if (prevSegment.angle == 270.0)
                    {
                        // Heading is West.
                        prevEndPoint.x -= turnStartAdj;
                        if (nextSegment.y > 0.0)
                        {
                            nextStartPoint.x = prevSegment.x - forwardAdj;
                            nextStartPoint.y = prevSegment.y + turnEndAdj;
                            nextStartPoint.angle = 0.0;
                        }
                        else
                        {
                            nextStartPoint.x = prevSegment.x - forwardAdj;
                            nextStartPoint.y = prevSegment.y - turnEndAdj;
                            nextStartPoint.angle = 180.0;
                        }
                        nextSegmentPoint.x = nextStartPoint.x;
                        nextSegmentPoint.y += prevSegment.y;
                    }
                    nextSegmentPoint.angle = nextStartPoint.angle;

                    // Create endpoint of the previous segment.
                    pathBuilder.append(
                        new TrcPose2D(startGridPose.x + prevEndPoint.x * gridCellSize,
                                      startGridPose.y + prevEndPoint.y * gridCellSize,
                                      prevEndPoint.angle));
                    // Create startpoint of the next segment.
                    pathBuilder.append(
                        new TrcPose2D(startGridPose.x + nextStartPoint.x * gridCellSize,
                                      startGridPose.y + nextStartPoint.y * gridCellSize,
                                      nextStartPoint.angle));
                    // Create the next segment point.
                    pathBuilder.append(
                        new TrcPose2D(startGridPose.x + nextSegmentPoint.x * gridCellSize,
                                      startGridPose.y + nextSegmentPoint.y * gridCellSize,
                                      nextSegmentPoint.angle));

                    gridDriveQueue.remove(nextSegment);
                    tracer.traceVerbose(
                        moduleName,
                        "Turn Adjustments: prevEndPoint=" + prevEndPoint +
                        ", nextStartPoint=" + nextStartPoint +
                        ", nextSegPoint=" + nextSegmentPoint +
                        " (QSize=" + gridDriveQueue.size() + ")");
                    prevSegment = nextSegmentPoint;
                    deferSegment = false;
                }
            }

            if (deferSegment)
            {
                // The last loop was a coalesce loop. It means we have one more segment to add to the path.
                pathBuilder.append(
                    new TrcPose2D(startGridPose.x + prevSegment.x * gridCellSize,
                                  startGridPose.y + prevSegment.y * gridCellSize,
                                  prevSegment.angle));
            }

            TrcPath path = pathBuilder.toRelativeStartPath();
            tracer.traceVerbose(moduleName, "GridDrivePath=" + path);
            callbackEvent.setCallback(this::driveDone, null);
            purePursuitDrive.start(moduleName, path, callbackEvent, 0.0);
        }
    }   //startGridDrive

    /**
     * This method is called when the PurePursuitDrive operation is done. It releases the ownership of the drive base.
     *
     * @param context not used.
     */
    private void driveDone(Object context)
    {
        driveBase.releaseExclusiveAccess(moduleName);
    }   //driveDone

    /**
     * This method determines if the next segment has a compatible heading with the previous segment. If not, the robot
     * will turn.
     *
     * @param prevSegment specifies the previous segment position.
     * @param nextSegment specifies the next segment position.
     * @return true if the robot will turn from the previous segment to the next segment, false otherwise.
     */
    private boolean willTurn(TrcPose2D prevSegment, TrcPose2D nextSegment)
    {
        return (nextSegment.x != 0.0 && (prevSegment.angle == 0.0 || prevSegment.angle == 180.0)) ||
               (nextSegment.y != 0.0 && (prevSegment.angle == 90.0 || prevSegment.angle == 270.0));
    }   //willTurn

}   //class TrcGridDrive
