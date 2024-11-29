package teamcode.autotasks;

import androidx.annotation.NonNull;

import teamcode.FtcAuto;
import teamcode.FtcTeleOp;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.Claw;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoPickupSpecimen extends TrcAutoTask<TaskAutoPickupSpecimen.State>
{
    private static final String moduleName = TaskAutoPickupSpecimen.class.getSimpleName();

    public enum State
    {
        GO_TO_SCORE_POSITION,
        SET_ELEVATOR_ARM,
        GRAB_SPECIMEN,
        RETRACT_ELEVATOR_ARM,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        TaskParams(FtcAuto.Alliance alliance)
        {
            this.alliance = alliance;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "alliance=" + alliance;
        }   //toString
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event;
    private final TrcEvent event2;

    private String currOwner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickupSpecimen(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        this.event = new TrcEvent(moduleName + ".event");
        this.event2 = new TrcEvent(moduleName + ".event2");
    }   //TaskAuto

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoPickupSpecimen(FtcAuto.Alliance alliance, TrcEvent completionEvent)
    {
        if (alliance == null)
        {
            // Caller is TeleOp, let's determine the alliance color by robot's location.
            alliance = robot.robotDrive.driveBase.getFieldPosition().y < 0.0?
                    FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        }

        TaskParams taskParams = new TaskParams(alliance);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(State.GO_TO_SCORE_POSITION, taskParams, completionEvent);
    }   //autoAssist

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        tracer.traceInfo(moduleName, "Canceling auto-assist.");
        stopAutoTask(false);
    }   //autoAssistCancel

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
                (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName));

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
                            ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
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
                            ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
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
        robot.robotDrive.cancel(currOwner);
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
            case GO_TO_SCORE_POSITION:
                //Path to pickup location
                robot.robotDrive.purePursuitDrive.start(currOwner, event2, 0.0,
                        robot.robotDrive.driveBase.getFieldPosition(), false, robot.robotInfo.profiledMaxVelocity,
                        robot.robotInfo.profiledMaxAcceleration, robot.adjustPoseByAlliance(RobotParams.Game.RED_OBSERVATION_ZONE_PICKUP, taskParams.alliance));
                //Set Elbow to pickup angle
                robot.elbow.setPosition(0,RobotParams.ElbowParams.PICKUP_SPECIMEN_POS,true,RobotParams.ElbowParams.POWER_LIMIT,event);
                //Position wrist and arm subsystems for pickup
                robot.wristArm.setWristArmPickupSamplePos(null);
                robot.clawServo.open();
                robot.wristRotational.setPosition(RobotParams.WristParamsRotational.MIDDLE_P0S);
                sm.waitForSingleEvent(event, State.SET_ELEVATOR_ARM);
                break;

            case SET_ELEVATOR_ARM:
                //Extend elevator to pickup position
                robot.elevator.setPosition(0, RobotParams.ElevatorParams.PICKUP_SPECIMEN_POS,true,RobotParams.ElevatorParams.POWER_LIMIT,event);
                sm.addEvent(event);
                sm.addEvent(event2);
                sm.waitForEvents(State.GRAB_SPECIMEN);
                break;

            case GRAB_SPECIMEN:
                robot.clawServo.close(event);
                sm.waitForSingleEvent(event, State.RETRACT_ELEVATOR_ARM);
                break;

            case RETRACT_ELEVATOR_ARM:
                //retract elevator "fire and forget"
                robot.elevator.setPosition(RobotParams.ElevatorParams.HIGH_CHAMBER_SCORE_POS);
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