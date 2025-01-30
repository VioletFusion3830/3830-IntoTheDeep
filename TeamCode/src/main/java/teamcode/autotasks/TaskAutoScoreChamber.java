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
        GO_TO_SCORE_POSITION,
        PUSH_SPECIMEN,
        CLIP_SPECIMEN,
        SCORE_CHAMBER,
        RETRACT_ELBOW,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        final TrcPose2D scorePose;
        final boolean cycle;

        TaskParams(
            FtcAuto.Alliance alliance, TrcPose2D scorePose, boolean cycle)
        {
            this.alliance = alliance;
            this.scorePose = scorePose;
            this.cycle = cycle;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return String.format(
                    Locale.US, "alliance=%s,scorePose=%s",
                    alliance, scorePose);
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
    public TaskAutoScoreChamber(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        this.event1 = new TrcEvent(moduleName + ".event1");
        this.event2 = new TrcEvent(moduleName + ".event2");
    }   //TaskAuto

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoScoreChamber(TrcPose2D scorePose, boolean cycle, TrcEvent completionEvent)
    {
        TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
        FtcAuto.Alliance alliance = robotPose.y < 0.0 ? FtcAuto.Alliance.RED_ALLIANCE : FtcAuto.Alliance.BLUE_ALLIANCE;
        if (scorePose == null)
        {
            boolean nearNetZone = alliance == FtcAuto.Alliance.RED_ALLIANCE ^ robotPose.x > 0.0;
             scorePose = nearNetZone ?
                    RobotParams.Game.RED_NET_CHAMBER_SCORE_POSE.clone() :
                    RobotParams.Game.RED_OBSERVATION_CHAMBER_SCORE_POSE.clone();
        }

        TaskParams taskParams = new TaskParams(alliance, scorePose, cycle);
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
                robot.arm.acquireExclusiveAccess(ownerName) &&
                robot.verticalWrist.acquireExclusiveAccess(ownerName));

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
                            ", arm=" + ownershipMgr.getOwner(robot.arm) +
                            ", verticalWrist=" + ownershipMgr.getOwner(robot.verticalWrist) + ").");
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
                            ", arm=" + ownershipMgr.getOwner(robot.arm) +
                            ", verticalWrist=" + ownershipMgr.getOwner(robot.verticalWrist) + ").");
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            robot.arm.releaseExclusiveAccess(currOwner);
            robot.verticalWrist.releaseExclusiveAccess(currOwner);
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
        //robot.elbowElevatorArm.cancel();
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
                //Set Subsystems to high chamber score position
                robot.elbowElevator.setPosition(RobotParams.ElbowParams.HIGH_CHAMBER_SCORE_POS, RobotParams.ElevatorParams.HIGH_CHAMBER_SCORE_POS, event1);
                robot.verticalWrist.setPosition(currOwner, 0.15, RobotParams.WristParamsVertical.HIGH_CHAMBER_SCORE_POS, null, 0);
                robot.arm.setPosition(currOwner, 0.15, RobotParams.ArmParams.HIGH_CHAMBER_SCORE_POS, null, 0);
                if(taskParams.cycle)
                {
                    robot.robotDrive.purePursuitDrive.start(
                            currOwner, event2, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                            robot.adjustPoseByAlliance(taskParams.scorePose, taskParams.alliance));
                }
                else
                {
                    robot.robotDrive.purePursuitDrive.start(
                            currOwner, event2, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                            robot.adjustPoseByAlliance(RobotParams.Game.RED_OBSERVATION_ZONE_CYCLE_SCORE_POSE, taskParams.alliance,false));
                }
                //Wait for completion
                sm.addEvent(event1);
                sm.addEvent(event2);
                sm.waitForEvents(State.SCORE_CHAMBER,true);
                break;

            case CLIP_SPECIMEN:
                //Clip specimen
                robot.elbowElevator.setPosition(null,RobotParams.ElevatorParams.HIGH_CHAMBER_CLIP_POS,event1);
                robot.arm.setPosition(currOwner,0,0.73,null,0);
                sm.waitForSingleEvent(event1,State.SCORE_CHAMBER);

            case SCORE_CHAMBER:
                //release specimen
                robot.clawGrabber.open(null,event1);
                sm.waitForSingleEvent(event1, State.RETRACT_ELBOW);
                break;

            case RETRACT_ELBOW:
                //retract elbow, arm, and elbow "fire and forget"
                robot.wristArm.setWristArmPickupSpecimenPos(currOwner,0,null);
                robot.elbowElevator.setPosition(RobotParams.ElbowParams.PICKUP_SPECIMEN_POS,RobotParams.ElevatorParams.PICKUP_SPECIMEN_POS,null);
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