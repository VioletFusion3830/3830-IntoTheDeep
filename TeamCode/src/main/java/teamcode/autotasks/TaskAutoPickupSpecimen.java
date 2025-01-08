package teamcode.autotasks;

import androidx.annotation.NonNull;

import teamcode.FtcAuto;
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
public class TaskAutoPickupSpecimen extends TrcAutoTask<TaskAutoPickupSpecimen.State>
{
    private static final String moduleName = TaskAutoPickupSpecimen.class.getSimpleName();

    public enum State
    {
        GO_TO_SCORE_POSITION,
        SET_ELEVATOR,
        GRAB_SPECIMEN,
        RETRACT_ELEVATOR_ARM,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        final boolean positionsSet;

        TaskParams(FtcAuto.Alliance alliance, boolean positionsSet)
        {
            this.alliance = alliance;
            this.positionsSet = positionsSet;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "alliance=" + alliance + ", positionsSet=" + positionsSet;
        }   //toString
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event1;
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
        this.event1 = new TrcEvent(moduleName + ".event");
        this.event2 = new TrcEvent(moduleName + ".event2");
    }   //TaskAuto

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoPickupSpecimen(FtcAuto.Alliance alliance,boolean positionsSet, TrcEvent completionEvent)
    {
        if (alliance == null)
        {
            // Caller is TeleOp, let's determine the alliance color by robot's location.
            alliance = robot.robotDrive.driveBase.getFieldPosition().y < 0.0?
                    FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        }

        TaskParams taskParams = new TaskParams(alliance, positionsSet);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(State.GO_TO_SCORE_POSITION, taskParams, completionEvent);
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
                (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName) &&
                        robot.verticalWrist.acquireExclusiveAccess(ownerName) &&
                        robot.arm.acquireExclusiveAccess(ownerName));

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
                            ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) +
                            ", wristVertical=" + ownershipMgr.getOwner(robot.verticalWrist) +
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
                            ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) +
                            ", wristVertical=" + ownershipMgr.getOwner(robot.verticalWrist) +
                            ", arm=" + ownershipMgr.getOwner(robot.arm) + ").");
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
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
        robot.robotDrive.cancel(currOwner);
        robot.elbow.cancel();
        robot.verticalWrist.cancel();
        robot.arm.cancel();
        robot.elevator.cancel();
        robot.clawGrabber.cancel();
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
            case GO_TO_SCORE_POSITION:
                //Path to pickup location
                if(!taskParams.positionsSet)
                {
                    robot.robotDrive.purePursuitDrive.start(currOwner, event1, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                            robot.adjustPoseByAlliance(RobotParams.Game.RED_OBSERVATION_ZONE_PICKUP, taskParams.alliance));
                    robot.elbowElevator.setPosition(true,RobotParams.ElbowParams.PICKUP_SPECIMEN_POS,RobotParams.ElevatorParams.PICKUP_SPECIMEN_POS-2, event2);
                    robot.wristArm.setWristArmPickupSpecimenPos(currOwner,0,null);
                    sm.addEvent(event1);
                    sm.addEvent(event2);
                    sm.waitForEvents(State.SET_ELEVATOR, true);
                }
                else
                {
                    sm.setState(State.GRAB_SPECIMEN);
                }
                break;

            case SET_ELEVATOR:
                robot.elbowElevator.setPosition(null, RobotParams.ElevatorParams.PICKUP_SPECIMEN_POS,event1);
                sm.waitForSingleEvent(event1, State.GRAB_SPECIMEN);
                break;

            case GRAB_SPECIMEN:
                robot.clawGrabber.close(null,0,event1);
                sm.waitForSingleEvent(event1, State.RETRACT_ELEVATOR_ARM);
                break;

            case RETRACT_ELEVATOR_ARM:
                //retract elevator & arm "fire and forget"
                robot.rotationalWrist.setPosition(RobotParams.WristParamsRotational.PARALLEL_SECONDARY_POS);
                robot.wristArm.setWristArmHighChamberScorePos(currOwner,0,null);
                robot.elevator.setPosition(currOwner,0.2,RobotParams.ElevatorParams.HIGH_CHAMBER_SCORE_POS,true,RobotParams.ElbowParams.POWER_LIMIT,null,0);
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