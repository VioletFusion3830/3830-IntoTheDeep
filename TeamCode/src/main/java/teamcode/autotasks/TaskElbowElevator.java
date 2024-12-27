package teamcode.autotasks;

import androidx.annotation.NonNull;

import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

public class TaskElbowElevator extends TrcAutoTask<TaskElbowElevator.State> {
    private static final String moduleName = TaskElbowElevator.class.getSimpleName();

    public enum State {
        SET_POSITION,
        RETRACT_ELEVATOR,
        SET_ELBOW_ANGLE,
        SET_ELEVATOR_POSITION,
        WAIT_FOR_COMPLETION,
        DONE
    }   //enum State

    private static class TaskParams {
        Double elevatorInitialPos;
        Double elbowAngle;
        Double elevatorPosition;

        TaskParams(Double elevatorInitialPos, Double elbowAngle, Double elevatorPosition) {
            this.elevatorInitialPos = elevatorInitialPos;
            this.elbowAngle = elbowAngle;
            this.elevatorPosition = elevatorPosition;
        }   //TaskParams

        @NonNull
        public String toString() {
            return "elevatorInitialPos = " + elevatorInitialPos + ",elbowPos=" + elbowAngle + ",elevatorPos=" + elevatorPosition;
        }   //toString
    }   //class TaskParams

    private final String ownerName;
    public final TrcMotor elbow;
    public final TrcMotor elevator;
    private final TrcEvent elbowEvent;
    private final TrcEvent elevatorEvent;
    private final TrcEvent elevatorEventInital;

    private String currOwner = null;
    private Double elevatorInitialPos = null;
    private boolean safeSequence = false;

    public TaskElbowElevator(String ownerName, TrcMotor elbow, TrcMotor elevator)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.elbow = elbow;
        this.elevator = elevator;
        this.elbowEvent = new TrcEvent(RobotParams.ElbowParams.SUBSYSTEM_NAME);
        this.elevatorEvent = new TrcEvent(RobotParams.ElevatorParams.SUBSYSTEM_NAME);
        this.elevatorEventInital = new TrcEvent(RobotParams.ElevatorParams.SUBSYSTEM_NAME);
    }   //TaskelevatorArm

    public void setPosition(
            boolean safeSequence,Double elevatorInitialPos, Double elbowAngle, Double elevatorPosition, TrcEvent completionEvent)
    {
        TaskParams taskParams = new TaskParams(elevatorInitialPos,elbowAngle, elevatorPosition);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        this.safeSequence = safeSequence;
        startAutoTask(State.SET_POSITION, taskParams, completionEvent);
    }   //setPosition

    public void setPosition(
            Double elevatorInitialPos, Double elbowAngle,  Double elevatorPosition, TrcEvent completionEvent)
    {
        setPosition(false, elevatorInitialPos, elbowAngle, elevatorPosition, completionEvent);
    }   //setPosition

    public void setPosition(
            boolean safeSequence,Double elbowAngle, Double elevatorPosition, TrcEvent completionEvent)
    {
        setPosition(safeSequence, null, elbowAngle, elevatorPosition, completionEvent);
    }   //setPosition

    public void setPosition(Double elbowAngle, Double elevatorPosition, TrcEvent completionEvent)
    {
        setPosition(false, null, elbowAngle, elevatorPosition, completionEvent);
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
                        elevator.acquireExclusiveAccess(ownerName);

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
                            ", elevator=" + ownershipMgr.getOwner(elevator) + ").");
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
                            ", elevator=" + ownershipMgr.getOwner(elevator) + ").");
            elbow.releaseExclusiveAccess(currOwner);
            elevator.releaseExclusiveAccess(currOwner);
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
                elevatorEventInital.clear();
                sm.setState(elevatorInitialPos != null? State.RETRACT_ELEVATOR: State.SET_ELBOW_ANGLE);
                break;

            case RETRACT_ELEVATOR:
                if (taskParams.elevatorInitialPos != null)
                {
                    // We retract the elevator first.
                    elevator.setPosition(
                            currOwner, 0.0, taskParams.elevatorInitialPos, true, RobotParams.ElevatorParams.POWER_LIMIT, elevatorEventInital, 0.0);
                    if (safeSequence) {
                        sm.waitForSingleEvent(elevatorEventInital, State.SET_ELBOW_ANGLE);
                    } else {
                        sm.setState(State.SET_ELBOW_ANGLE);
                    }
                }
                else
                {
                    // Caller did not provide initial elevator position, skip this state.
                    elevatorEventInital.signal();
                    sm.setState(State.SET_ELBOW_ANGLE);

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
                    sm.setState(State.DONE);
                }
                else
                {
                    sm.addEvent(elbowEvent);
                    sm.addEvent(elevatorEvent);
                    sm.addEvent(elevatorEventInital);
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
