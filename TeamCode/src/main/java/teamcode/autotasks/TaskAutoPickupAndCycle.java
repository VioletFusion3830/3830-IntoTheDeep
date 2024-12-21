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

package teamcode.autotasks;

import androidx.annotation.NonNull;

import java.util.Locale;

import teamcode.FtcDashboard;
import teamcode.FtcTeleOp;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoPickupAndCycle extends TrcAutoTask<TaskAutoPickupAndCycle.State>
{
    private static final String moduleName = TaskAutoPickupAndCycle.class.getSimpleName();

    public enum State
    {
        SET_ARM_POS,
        GRAB,
        RESET_ARM_POS,
        CYCLE_BASKET,
        SCORE_SAMPLE,
        CYCLE_INTAKE,
        DONE
    }   //enum State

    private static class TaskParams
    {
        boolean cycle;
        TaskParams(boolean cycle)
        {
            this.cycle = cycle;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return String.format(
                    Locale.US, "cycle=%s", cycle);
        }   //toString

    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private TrcEvent event;

    private String currOwner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickupAndCycle(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        event = new TrcEvent(moduleName);
    }   //TaskAuto

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoPickup(boolean cycle, TrcEvent completionEvent)
    {
        FtcTeleOp.isClawGrabbing = true;
        TaskAutoPickupAndCycle.TaskParams taskParams = new TaskAutoPickupAndCycle.TaskParams(cycle);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(State.SET_ARM_POS, taskParams, completionEvent);
    }   //autoAssist

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called by the super class to acquire ownership of all subsystems involved in the auto-assist
     * operation. This is typically done before starting an auto-assist operation.
     *
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership()
    {
        boolean success = ownerName == null ||
                (robot.arm.acquireExclusiveAccess(ownerName));

        if (success)
        {
            currOwner = ownerName;
            tracer.traceInfo(moduleName, "Successfully acquired subsystem ownerships.");
        }
        else
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceWarn(
                    moduleName,
                    "Failed to acquire subsystem ownership (currOwner=" + currOwner+
                            ", arm=" + ownershipMgr.getOwner(robot.arm) + ").");
            releaseSubsystemsOwnership();
        }

        return success;
    }   //acquireSubsystemsOwnership

    /**
     * This method is called by the super class to release ownership of all subsystems involved in the auto-assist
     * operation. This is typically done if the auto-assist operation is completed or canceled.
     */
    @Override
    protected void releaseSubsystemsOwnership()
    {
        if (ownerName != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                    moduleName,
                    "Releasing subsystem ownership (currOwner=" + currOwner +
                            ", arm=" + ownershipMgr.getOwner(robot.arm) + ").");

            robot.arm.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     */
    @Override
    protected void stopSubsystems()
    {
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        robot.arm.cancel();
        robot.clawServo.cancel();
        robot.verticalWrist.cancel();
        robot.rotationalWrist.cancel();
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
            Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case SET_ARM_POS:
                robot.wristArm.setWristArmPosition(currOwner,robot.armElevatorScaling()-0.1,robot.vWristElevatorScaling()-0.08,0.17,event);
                sm.waitForSingleEvent(event,State.GRAB);
                break;

            case GRAB:
                robot.clawServo.close(currOwner, 0, event);
                sm.waitForSingleEvent(event, State.RESET_ARM_POS);
                break;

            case RESET_ARM_POS:
                FtcTeleOp.isClawGrabbing = false;
                if(taskParams.cycle && robot.clawServo.hasObject())
                {
                    robot.wristArm.setWristArmHighChamberScorePos(currOwner, 0.2,null);
                    robot.rotationalWrist.setPosition(RobotParams.WristParamsRotational.MIDDLE_P0S);
                    sm.setState(State.CYCLE_BASKET);
                }
                else
                {
                    robot.wristArm.setWristArmPosition(currOwner,robot.armElevatorScaling(),robot.vWristElevatorScaling(),.2,null);
                    sm.setState(State.DONE);
                }
                break;

            case CYCLE_BASKET:
                robot.elbowElevator.setPosition(true, RobotParams.ElevatorParams.MIN_POS, RobotParams.ElbowParams.BASKET_SCORE_POS, RobotParams.ElevatorParams.HIGH_BASKET_SCORE_POS, event);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            case SCORE_SAMPLE:
                robot.clawServo.open(currOwner, 0, event);
                sm.waitForSingleEvent(event, State.CYCLE_INTAKE);

            case CYCLE_INTAKE:
                robot.wristArm.setWristArmPickupSamplePos(0.2, null);
                robot.elbowElevator.setPosition(true, RobotParams.ElevatorParams.MIN_POS, RobotParams.ElbowParams.PICKUP_SAMPLE_POS, 26.0, event);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAuto
