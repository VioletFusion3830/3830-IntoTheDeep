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
public class TaskAutoPickup extends TrcAutoTask<TaskAutoPickup.State>
{
    private static final String moduleName = TaskAutoPickup.class.getSimpleName();

    public enum State
    {
        SET_ARM_POS,
        GRAB,
        RESET_ARM_POS,
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
    private TrcEvent event2;

    private String currOwner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickup(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        event = new TrcEvent(moduleName);
        event2 = new TrcEvent(moduleName);
    }   //TaskAuto

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoPickup(TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "event=" + completionEvent);
        startAutoTask(State.SET_ARM_POS, new TaskParams(), completionEvent);
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
                (robot.arm.acquireExclusiveAccess(ownerName) &&
                 robot.clawServo.acquireExclusiveAccess(currOwner) &&
                 robot.wristVertical.acquireExclusiveAccess(currOwner));

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
                            ", clawServo=" + ownershipMgr.getOwner(robot.clawServo)+
                            ", vWrist=" + ownershipMgr.getOwner(robot.wristVertical)+").");
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
                            ", clawServo=" + ownershipMgr.getOwner(robot.clawServo)+
                            ", vWrist=" + ownershipMgr.getOwner(robot.wristVertical)+").");

            robot.arm.releaseExclusiveAccess(currOwner);
            robot.clawServo.releaseExclusiveAccess(currOwner);
            robot.wristVertical.releaseExclusiveAccess(currOwner);
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
        robot.wristVertical.cancel();
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
                FtcTeleOp.isClawGrabbing = true;
                double armPos = RobotParams.ArmParams.SAMPLE_PICKUP_MODE_START -
                        (RobotParams.ArmParams.SAMPLE_PICKUP_MODE_SCALE * ((robot.elevator.getPosition() - RobotParams.ElevatorParams.MIN_POS)/18));
                double vWristPos = RobotParams.WristParamsVertical.SAMPLE_PICKUP_MODE_START -
                        (RobotParams.WristParamsVertical.SAMPLE_PICKUP_MODE_SCALE * ((robot.elevator.getPosition() - RobotParams.ElevatorParams.MIN_POS)/18));
                robot.wristVertical.setPosition(currOwner,0,vWristPos+ FtcDashboard.ServoTune.ServoB,event2,.3);
                robot.arm.setPosition(currOwner,0, armPos+ FtcDashboard.ServoTune.ServoA,event,0.3);
                sm.waitForEvents(State.GRAB);
                break;

            case GRAB:
                robot.clawServo.close(currOwner,event);
                sm.waitForEvents(State.RESET_ARM_POS,.3);
                break;

            case RESET_ARM_POS:
                FtcTeleOp.isClawGrabbing = false;
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
