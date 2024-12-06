package teamcode.autotasks;

import androidx.annotation.NonNull;

import java.util.Locale;

import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoScoreChamber extends TrcAutoTask<TaskAutoScoreChamber.State>
{
    private static final String moduleName = TaskAutoScoreChamber.class.getSimpleName();

    public enum State
    {
        SET_SUBSYSTEMS,
        GO_TO_SCORE_POSITION,
        CLIP_SPECIMEN,
        SCORE_CHAMBER,
        RETRACT_ELBOW,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        final TrcPose2D scorePose;
        final boolean noDrive;

        TaskParams(
            FtcAuto.Alliance alliance, TrcPose2D scorePose, boolean noDrive)
        {
            this.alliance = alliance;
            this.scorePose = scorePose;
            this.noDrive = noDrive;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return String.format(
                    Locale.US, "alliance=%s,scorePose=%s,noDrive=%s",
                    alliance, scorePose, noDrive);
        }   //toString

    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event1;
    private final TrcEvent event2;
    private final TrcEvent event3;

    private String currOwner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoScoreChamber(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        this.event1 = new TrcEvent(moduleName + ".event1");
        this.event2 = new TrcEvent(moduleName + ".event2");
        this.event3 = new TrcEvent(moduleName + ".event3");
    }   //TaskAuto

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoScoreChamber(TrcPose2D scorePose, boolean noDrive, TrcEvent completionEvent)
    {
        TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
        FtcAuto.Alliance alliance = robotPose.y < 0.0? FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        boolean nearNetZone = alliance == FtcAuto.Alliance.RED_ALLIANCE ^ robotPose.x > 0.0;
        if(scorePose == null)
        {
            scorePose = nearNetZone ?
                    RobotParams.Game.RED_NET_CHAMBER_SCORE_POSE.clone() :
                    RobotParams.Game.RED_OBSERVATION_CHAMBER_SCORE_POSE.clone();

            if (robotPose.x >= -RobotParams.Game.CHAMBER_MAX_SCORE_POS_X &&
                    robotPose.x <= RobotParams.Game.CHAMBER_MAX_SCORE_POS_X) {
                // If robot current position is within the chamber zone, use its X position.
                scorePose.x = robotPose.x;
            }
        }

        TaskParams taskParams = new TaskParams(alliance, scorePose,noDrive);
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
                        robot.elbow.acquireExclusiveAccess(ownerName) &&
                        robot.wristVertical.acquireExclusiveAccess(ownerName) &&
                        robot.arm.acquireExclusiveAccess(ownerName) &&
                        robot.elevator.acquireExclusiveAccess(ownerName) &&
                        //robot.clawServo.acquireExclusiveAccess(ownerName) &&
                        robot.wristRotational.acquireExclusiveAccess(ownerName));

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
                            ", elbow=" + ownershipMgr.getOwner(robot.elbow) +
                            ", wristVertical=" + ownershipMgr.getOwner(robot.wristVertical) +
                            ", arm=" + ownershipMgr.getOwner(robot.arm) +
                            ", elevator=" + ownershipMgr.getOwner(robot.elevator) +
                            //", clawServo=" + ownershipMgr.getOwner(robot.clawServo) +
                            ", wristRotational=" + ownershipMgr.getOwner(robot.wristRotational) + ").");
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
                            ", elbow=" + ownershipMgr.getOwner(robot.elbow) +
                            ", wristVertical=" + ownershipMgr.getOwner(robot.wristVertical) +
                            ", arm=" + ownershipMgr.getOwner(robot.arm) +
                            ", elevator=" + ownershipMgr.getOwner(robot.elevator) +
                            //", clawServo=" + ownershipMgr.getOwner(robot.clawServo) +
                            ", wristRotational=" + ownershipMgr.getOwner(robot.wristRotational) + ").");
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            robot.elbow.releaseExclusiveAccess(currOwner);
            robot.wristVertical.releaseExclusiveAccess(currOwner);
            robot.arm.releaseExclusiveAccess(currOwner);
            robot.elevator.releaseExclusiveAccess(currOwner);
            //robot.clawServo.releaseExclusiveAccess(currOwner);
            robot.wristRotational.releaseExclusiveAccess(currOwner);
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
        robot.wristVertical.cancel();
        robot.arm.cancel();
        robot.elevator.cancel();
        robot.clawServo.cancel();
        robot.wristRotational.cancel();
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
            case SET_SUBSYSTEMS:
                if(!taskParams.noDrive){
                    //Path to pickup location
                    robot.robotDrive.purePursuitDrive.start(
                            currOwner, event1, 0.0,
                            robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.adjustPoseByAlliance(taskParams.scorePose, taskParams.alliance));
                }
                if(robot.elevator.getPosition() > RobotParams.ElevatorParams.HIGH_CHAMBER_SCORE_POS)
                {
                    robot.elevator.setPosition(currOwner,0,RobotParams.ElevatorParams.HIGH_CHAMBER_SCORE_POS,true,RobotParams.ElevatorParams.POWER_LIMIT,event3,3);
                    sm.waitForSingleEvent(event2, State.GO_TO_SCORE_POSITION);
                }
                else
                {
                    sm.setState(State.GO_TO_SCORE_POSITION);
                }
                break;

            case GO_TO_SCORE_POSITION:
                //Set Elbow and elevator to pickup positions
                robot.elbow.setPosition(currOwner,0,RobotParams.ElbowParams.HIGH_CHAMBER_SCORE_POS,true,RobotParams.ElbowParams.POWER_LIMIT,event2,3);
                if(robot.elevator.getPosition() > RobotParams.ElevatorParams.HIGH_CHAMBER_SCORE_POS)
                {
                    robot.elevator.setPosition(currOwner, 0, RobotParams.ElevatorParams.HIGH_CHAMBER_SCORE_POS, true, RobotParams.ElevatorParams.POWER_LIMIT, event3, 3);
                }
                //Position wrist and arm subsystems for deposit
                robot.arm.setPosition(currOwner,0.2,RobotParams.ArmParams.HIGH_CHAMBER_SCORE_POS,null,3);
                robot.wristVertical.setPosition(currOwner,0.2,RobotParams.WristParamsVertical.HIGH_CHAMBER_SCORE_POS,null,2);
                robot.wristRotational.setPosition(currOwner,0,RobotParams.WristParamsRotational.MIDDLE_POS2,null,2);
                //Wait for completion
                sm.addEvent(event1);
                sm.addEvent(event2);
                sm.addEvent(event3);
                sm.waitForEvents(State.CLIP_SPECIMEN,true);
                break;

            case CLIP_SPECIMEN:
                //Lower elevator to clip specimen
                robot.arm.setPosition(currOwner,0.2,7,null,3);
                robot.elevator.setPosition(currOwner,0,RobotParams.ElevatorParams.MIN_POS_ELBOW_UP,true,RobotParams.ElevatorParams.POWER_LIMIT,event1,3);
                sm.waitForSingleEvent(event1, State.SCORE_CHAMBER);
                break;

            case SCORE_CHAMBER:
                //release specimen
                robot.clawServo.open(null,event1);
                sm.waitForSingleEvent(event1, State.RETRACT_ELBOW);
                break;

            case RETRACT_ELBOW:
                //retract elbow, arm, and elbow "fire and forget"
                robot.elevator.setPosition(null,0,15,true,RobotParams.ElevatorParams.POWER_LIMIT,null,3);
                robot.elbow.setPosition(null,0.3,RobotParams.ElbowParams.PICKUP_SPECIMEN_POS,true,RobotParams.ElbowParams.POWER_LIMIT,null,3);
                sm.setState(State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoScoreChamber