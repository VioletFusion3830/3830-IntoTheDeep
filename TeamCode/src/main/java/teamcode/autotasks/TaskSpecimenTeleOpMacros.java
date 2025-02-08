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
public class TaskSpecimenTeleOpMacros extends TrcAutoTask<TaskSpecimenTeleOpMacros.State>
{
    private static final String moduleName = TaskSpecimenTeleOpMacros.class.getSimpleName();

    public enum State
    {
        GO_TO_PICKUP_SPECIMEN_POSITION,
        GRAB_SPECIMEN,
        GO_TO_SCORING_SPECIMEN_POSITION,
        CLIP_SPECIMEN,
        SCORE_SPECIMEN,
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
    private final TrcEvent event1;
    private final TrcEvent event2;

    private String currOwner = null;
    double timesCycled = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskSpecimenTeleOpMacros(String ownerName, Robot robot)
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
    public void autoClipSpecamins(FtcAuto.Alliance alliance, TrcEvent completionEvent)
    {
        if (alliance == null)
        {
            // Caller is TeleOp, let's determine the alliance color by robot's location.
            alliance = robot.robotDrive.driveBase.getFieldPosition().y < 0.0?
                    FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        }

        timesCycled = 0;
        TaskParams taskParams = new TaskParams(alliance);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(State.GO_TO_PICKUP_SPECIMEN_POSITION, taskParams, completionEvent);
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
            Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop) {
        TaskParams taskParams = (TaskParams) params;

        switch (state) {
            case GO_TO_PICKUP_SPECIMEN_POSITION:
                //Path to pickup location
                if(timesCycled == 0)
                {
                    robot.robotDrive.purePursuitDrive.start(currOwner, event1, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                            robot.adjustPoseByAlliance(RobotParams.Game.RED_OBSERVATION_ZONE_PICKUP_START, taskParams.alliance));
                }
                else
                {
                    robot.robotDrive.purePursuitDrive.start(currOwner, event1, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                            robot.adjustPoseByAlliance(RobotParams.Game.RED_OBSERVATION_ZONE_PICKUP, taskParams.alliance, false));
                }
                robot.elbowElevator.setPosition(RobotParams.ElbowParams.PICKUP_SPECIMEN_POS, RobotParams.ElevatorParams.PICKUP_SPECIMEN_POS, event2);
                    robot.wristArm.setWristArmPickupSpecimenPos(currOwner, 0, null);
                    robot.rotationalWrist.setPosition(null, 0, RobotParams.WristParamsRotational.PARALLEL_BASE_P0S, null, 0);
                    sm.addEvent(event1);
                    sm.addEvent(event2);
                    sm.waitForEvents(State.GRAB_SPECIMEN, true);
                break;

            case GRAB_SPECIMEN:
                robot.clawGrabber.close(null,0,event1);
                sm.waitForSingleEvent(event1, State.GO_TO_SCORING_SPECIMEN_POSITION);
                break;

            case GO_TO_SCORING_SPECIMEN_POSITION:
                //Path to scoring location
                robot.robotDrive.purePursuitDrive.start(
                        currOwner, event2, 0.0, false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        robot.adjustPoseByAlliance(RobotParams.Game.RED_OBSERVATION_CHAMBER_SCORE_POSE, taskParams.alliance));
                robot.elbowElevator.setPosition(RobotParams.ElbowParams.HIGH_CHAMBER_SCORE_POS, RobotParams.ElevatorParams.HIGH_CHAMBER_SCORE_POS, event1);
                robot.verticalWrist.setPosition(currOwner, 0, RobotParams.WristParamsVertical.HIGH_CHAMBER_SCORE_POS, null, 0);
                robot.arm.setPosition(currOwner, 0, RobotParams.ArmParams.HIGH_CHAMBER_SCORE_POS, null, 0);
                robot.rotationalWrist.setPosition(null, 0, RobotParams.WristParamsRotational.PARALLEL_BASE_P0S, null, 0);
                sm.addEvent(event1);
                sm.addEvent(event2);
                sm.waitForEvents(State.CLIP_SPECIMEN,true);
                break;

            case CLIP_SPECIMEN:
                //Clip specimen
                robot.elbowElevator.setPosition(RobotParams.ElbowParams.HIGH_CHAMBER_SCORE_POS, RobotParams.ElevatorParams.HIGH_CHAMBER_CLIP_POS, event1);
                robot.wristArm.setWristArmPosition(currOwner,0.75,0.25,0,null);
                sm.waitForSingleEvent(event1, State.SCORE_SPECIMEN);
                break;

            case SCORE_SPECIMEN:
                //release specimen
                robot.clawGrabber.open(null,event1);
                sm.waitForSingleEvent(event1, State.GO_TO_PICKUP_SPECIMEN_POSITION);
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;

        }
    }   //runTaskState

}   //class TaskAuto