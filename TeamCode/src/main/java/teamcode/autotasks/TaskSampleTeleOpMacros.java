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
public class TaskSampleTeleOpMacros extends TrcAutoTask<TaskSampleTeleOpMacros.State>
{
    private static final String moduleName = TaskSampleTeleOpMacros.class.getSimpleName();

    public enum State
    {
        SET_ARM_POS,
        GRAB,
        RESET_ARM_POS,
        SET_SUBSYSTEMS_SAMPLE_SCORE_POS,
        SET_ARM_SAMPLE_SCORE_POS,
        SET_ARM_SAMPLE_PICKUP_POS,
        SET_PICKUP_SCALING,
        DONE
    }   //enum State

    private static class TaskParams
    {
        TaskParams()
        {
        }   //TaskParams
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
    public TaskSampleTeleOpMacros(String ownerName, Robot robot)
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
    public void autoPickSample(TrcEvent completionEvent)
    {
        FtcTeleOp.isClawGrabbing = true;
        startAutoTask(State.SET_ARM_POS, null, completionEvent);
    }   //autoAssist

    public void autoSetSubsystemsSampleScorePos(TrcEvent completionEvent)
    {
        FtcTeleOp.isSamplePickupMode = false;
        startAutoTask(State.SET_SUBSYSTEMS_SAMPLE_SCORE_POS, null, completionEvent);
    }   //autoAssist

    public void autoSetSubsystemsSamplePickupPos(TrcEvent completionEvent)
    {
        startAutoTask(State.SET_ARM_SAMPLE_PICKUP_POS, null, completionEvent);
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
                            ", arm=" + ownershipMgr.getOwner(robot.arm) +
                            ", verticalWrist " + ownershipMgr.getOwner(robot.arm) + ").");
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
                            ", arm=" + ownershipMgr.getOwner(robot.arm) +
                            ", verticalWrist " + ownershipMgr.getOwner(robot.verticalWrist) + ").");
            robot.verticalWrist.releaseExclusiveAccess(currOwner);
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
        robot.clawGrabber.cancel();
        robot.verticalWrist.cancel();
        robot.rotationalWrist.cancel();
        robot.elbowElevator.cancel();
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
            //Auto-assisted Pickup Sample
            case SET_ARM_POS:
                robot.wristArm.setWristArmPickupSamplePos(currOwner,.1, event);
                sm.waitForSingleEvent(event,State.RESET_ARM_POS);
                break;

            case GRAB:
                robot.clawGrabber.close(null, 0, event);
                sm.waitForSingleEvent(event, State.RESET_ARM_POS);
                break;

            case RESET_ARM_POS:
                FtcTeleOp.isClawGrabbing = false;
                robot.wristArm.setWristArmPickupReadySamplePos(currOwner,0,null);
                sm.setState(State.DONE);
                break;

            //Auto-assists set Subsystems Score Sample
            case SET_SUBSYSTEMS_SAMPLE_SCORE_POS:
                if(robot.elevator.getPosition() > 15 && robot.elbow.getPosition() < 80)
                {
                    robot.elbowElevator.setPosition(RobotParams.ElbowParams.BASKET_SCORE_POS, RobotParams.ElevatorParams.HIGH_BASKET_SCORE_POS, event);
                }
                else
                {
                    robot.elbowElevator.setPosition(false,RobotParams.ElevatorParams.MIN_POS, RobotParams.ElbowParams.BASKET_SCORE_POS, RobotParams.ElevatorParams.HIGH_BASKET_SCORE_POS,true,false, event);
                }
                robot.rotationalWrist.setPosition(null,0, RobotParams.WristParamsRotational.PARALLEL_BASE_P0S, null, 0);
                sm.waitForSingleEvent(event, State.SET_ARM_SAMPLE_SCORE_POS);
                break;

            case SET_ARM_SAMPLE_SCORE_POS:
                robot.wristArm.setWristArmBasketScorePos(currOwner,0.1, event);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            //Auto-assists set Subsystems pickup Sample
            case SET_ARM_SAMPLE_PICKUP_POS:
                robot.wristArm.setWristArmPosition(currOwner,0.5, 0.5,0,null);
                robot.elbowElevator.setPosition(RobotParams.ElevatorParams.PICKUP_SAMPLE_POS, RobotParams.ElbowParams.PICKUP_SAMPLE_POS,null,true, event);
                sm.waitForSingleEvent(event, State.SET_PICKUP_SCALING);
                break;

            case SET_PICKUP_SCALING:
                robot.wristArm.setWristArmPickupReadySamplePos(currOwner,0, null);
                FtcTeleOp.isSamplePickupMode = true;
                sm.setState(State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAuto
