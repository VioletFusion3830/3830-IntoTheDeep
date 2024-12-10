package teamcode.autotasks;

import androidx.annotation.NonNull;

import teamcode.Robot;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

public class TaskElbowElevatorArm extends TrcAutoTask<TaskElbowElevatorArm.State> {
    private static final String moduleName = TaskElbowElevatorArm.class.getSimpleName();

    public enum State {
        SET_POSITION,
        RETRACT_ELEVATOR,
        SET_WRIST_ARM_POSITIONS,
        SET_ELBOW_ANGLE,
        SET_ELEVATOR_POSITION,
        WAIT_FOR_COMPLETION,
        DONE
    }   //enum State

    private static class TaskParams {
        Double elevatorRetractionPos;
        Double elbowAngle;
        Double elevatorPosition;
        Double armPosition;
        Double verticalWristAngle;
        Double rotationalWristAngle;

        TaskParams(Double elevatorRetractionPos, Double elbowAngle, Double elevatorPosition, Double armPosition, Double verticalWristAngle, Double rotationalWristAngle) {
            this.elbowAngle = elbowAngle;
            this.elevatorPosition = elevatorPosition;
            this.armPosition = armPosition;
            this.verticalWristAngle = verticalWristAngle;
            this.rotationalWristAngle = rotationalWristAngle;
        }   //TaskParams

        @NonNull
        public String toString() {
            return "elevatorRetractionPos = " + elevatorRetractionPos + ",elbowPos=" + elbowAngle + ",elevatorPos=" + elevatorPosition + ",armPos=" + armPosition + ",verticalWristAngle=" + verticalWristAngle + ",rotationalWristAngle=" + rotationalWristAngle;
        }   //toString
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    public final TrcMotor elbow;
    public final TrcMotor elevator;
    private final TrcEvent elbowEvent;
    private final TrcEvent elevatorEvent;
    private final TrcEvent armEvent;

    private String currOwner = null;
    private Double elevatorRetractionPos = null;
    private boolean safeSequence = false;

    public TaskElbowElevatorArm(String ownerName, Robot robot, TrcMotor elbow, TrcMotor elevator)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        this.elbow = elbow;
        this.elevator = elevator;
        this.elbowEvent = new TrcEvent(RobotParams.ElbowParams.SUBSYSTEM_NAME);
        this.elevatorEvent = new TrcEvent(RobotParams.ElevatorParams.SUBSYSTEM_NAME);
        this.armEvent = new TrcEvent(moduleName);
    }   //TaskelevatorArm

    public void setPosition(
            boolean safeSequence,Double elevatorRetractionPos, Double elbowAngle, Double elevatorPosition,Double armPosition, Double verticalWristAngle, Double rotationalWristAngle, TrcEvent completionEvent)
    {
        TaskParams taskParams = new TaskParams(elevatorRetractionPos,elbowAngle, elevatorPosition, armPosition, verticalWristAngle, rotationalWristAngle);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        this.elevatorRetractionPos = elevatorRetractionPos;
        this.safeSequence = safeSequence;
        startAutoTask(State.SET_POSITION, taskParams, completionEvent);
    }   //setPosition

    /**
     * This method cancels the elevatorArm AutoTask.
     */
    public void cancel()
    {
        tracer.traceInfo(moduleName, "Canceling AutoTask");
        stopAutoTask(false);
        stopSubsystems();
    }   //cancel

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
                elbow.acquireExclusiveAccess(ownerName) &&
                        elevator.acquireExclusiveAccess(ownerName) &&
                        robot.arm.acquireExclusiveAccess(ownerName);

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
                    "Failed to acquire subsystem ownership (currOwner=" + currOwner +
                            ", elbow=" + ownershipMgr.getOwner(elbow) +
                            ", elevator=" + ownershipMgr.getOwner(elevator) + 
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
                            ", elbow=" + ownershipMgr.getOwner(elbow) +
                            ", elevator=" + ownershipMgr.getOwner(elevator) + 
                            ", arm=" + ownershipMgr.getOwner(robot.arm) + ").");
            elbow.releaseExclusiveAccess(currOwner);
            elevator.releaseExclusiveAccess(currOwner);
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
        elbow.cancel();
        elevator.cancel();
        robot.arm.cancel();
        robot.rotationalWrist.cancel();
        robot.verticalWrist.cancel();
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
            case SET_POSITION:
                elbowEvent.clear();
                elevatorEvent.clear();
                armEvent.clear();
                sm.setState(elevatorRetractionPos != null? State.RETRACT_ELEVATOR: State.SET_WRIST_ARM_POSITIONS);
                break;

            case RETRACT_ELEVATOR:
                // We retract the elevator first.
                elevator.setPosition(
                            currOwner, 0.0, taskParams.elevatorRetractionPos, true, RobotParams.ElevatorParams.POWER_LIMIT, elevatorEvent, 0.0);
                    sm.waitForSingleEvent(elbowEvent, State.SET_WRIST_ARM_POSITIONS);
                break;

            case SET_WRIST_ARM_POSITIONS:
                // set wrist and arm positions.
                // set arm armEvent the longest task time
                if(taskParams.rotationalWristAngle != null)
                {
                    robot.rotationalWrist.setPosition(currOwner, 0.0, taskParams.rotationalWristAngle, armEvent, 0.3);
                }
                if(taskParams.verticalWristAngle != null)
                {
                    robot.verticalWrist.setPosition(currOwner, 0.0, taskParams.verticalWristAngle, armEvent, 0.4);
                }
                if(taskParams.armPosition != null)
                {
                    robot.arm.setPosition(currOwner, 0.0, taskParams.armPosition, armEvent, 0.5);
                }
                break;

            case SET_ELBOW_ANGLE:
                if (taskParams.elbowAngle != null)
                {
                    // Setting target elbow angle
                    elbow.setPosition(
                            currOwner, 0.0, taskParams.elbowAngle, true, RobotParams.ElevatorParams.POWER_LIMIT, elbowEvent, 1.5);
                    if (safeSequence)
                    {
                        //preform safe sequence, so wait for elbow event.
                        sm.waitForSingleEvent(elbowEvent, State.SET_ELEVATOR_POSITION);
                    }
                    else
                    {
                        // Not performing safe sequence, so don't wait.
                        sm.setState(State.SET_ELEVATOR_POSITION);
                    }
                }
                else
                {
                    // Caller did not provide elbow angle, skip this state.
                    elbowEvent.signal();
                    sm.setState(State.SET_ELEVATOR_POSITION);
                }
                break;

            case SET_ELEVATOR_POSITION:
                if (taskParams.elevatorPosition != null)
                {
                    // Set target extender position.
                    elevator.setPosition(
                            currOwner, 0.0, taskParams.elevatorPosition, true, RobotParams.ElevatorParams.POWER_LIMIT, elevatorEvent, 2.0);
                    if (safeSequence)
                    {
                        // Performing safe sequence, so wait for elevator event.
                        sm.waitForSingleEvent(elevatorEvent, State.WAIT_FOR_COMPLETION);
                    }
                    else
                    {
                        // Not performing safe sequence, so don't wait.
                        sm.setState(State.WAIT_FOR_COMPLETION);
                    }
                }
                else
                {
                    // We are not setting extender position, we are done.
                    elevatorEvent.signal();
                    sm.setState(State.WAIT_FOR_COMPLETION);
                }
                break;

            case WAIT_FOR_COMPLETION:
                if (safeSequence)
                {
                    // If we performed safe sequence and came here, it means both events are already signaled.
                    // Wait for armEvent because does not support safe sequence.
                    sm.waitForSingleEvent(armEvent, State.DONE);
                }
                else
                {
                    sm.addEvent(elbowEvent);
                    sm.addEvent(elevatorEvent);
                    sm.addEvent(armEvent);
                    // Don't clear the events.
                    sm.waitForEvents(State.DONE, false, true);
                }
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
}   //class TaskExtenderArm
