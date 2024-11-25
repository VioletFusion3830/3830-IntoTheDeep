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
        SET_ELEVATOR_ARM,
        LOWER_ELBOW,
        SCORE_CHAMBER,
        RETRACT_ELEVATOR_ARM,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        final TrcPose2D scorePose;
        final double elbowAngle;
        final double elevatorPos;
        final double wristArmPos;
        final boolean noDrive;

        TaskParams(
            FtcAuto.Alliance alliance, TrcPose2D scorePose, double elbowAngle,
            double elevatorPos, double wristArmPos, boolean noDrive)
        {
            this.alliance = alliance;
            this.scorePose = scorePose;
            this.elbowAngle = elbowAngle;
            this.elevatorPos = elevatorPos;
            this.wristArmPos = wristArmPos;
            this.noDrive = noDrive;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return String.format(
                    Locale.US, "alliance=%s,scorePose=%s,elbowPos=%.1f, elevatorPos=%.1f,wristPos=%.3f,noDrive=%s",
                    alliance, scorePose, elbowAngle, elevatorPos, wristArmPos, noDrive);
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
    public TaskAutoScoreChamber(String ownerName, Robot robot)
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
    public void autoScoreChamber(Robot.ScoreHeight scoreHeight, boolean noDrive, TrcEvent completionEvent)
    {
        TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
        FtcAuto.Alliance alliance = robotPose.y < 0.0? FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        boolean nearNetZone = alliance == FtcAuto.Alliance.RED_ALLIANCE ^ robotPose.x > 0.0;
        TrcPose2D scorePose = nearNetZone?
                RobotParams.Game.RED_NET_CHAMBER_SCORE_POSE.clone():
                RobotParams.Game.RED_OBSERVATION_CHAMBER_SCORE_POSE.clone();
        double elbowAngle, elevatorPos, wristArmPos;

//        if (robotPose.x >= -RobotParams.Game.CHAMBER_MAX_SCORE_POS_X &&
//                robotPose.x <= RobotParams.Game.CHAMBER_MAX_SCORE_POS_X)
//        {
//            // If robot current position is within the chamber zone, use its X position.
//            scorePose.x = robotPose.x;
//        }
//
//        if (scoreHeight == Robot.ScoreHeight.LOW)
//        {
//            elbowAngle = RobotParams.ElbowParams.LOW_CHAMBER_SCORE_POS;
//            elevatorPos = Extender.Params.LOW_CHAMBER_SCORE_POS;
//            wristArmPos = Wrist.Params.LOW_CHAMBER_SCORE_POS;
//        }
//        else
//        {
//            elbowAngle = Elbow.Params.HIGH_CHAMBER_SCORE_POS;
//            extenderPos = Extender.Params.HIGH_CHAMBER_SCORE_POS;
//            wristPos = Wrist.Params.HIGH_CHAMBER_SCORE_POS;
//        }

        tracer.traceInfo(moduleName, "event=" + completionEvent);
//        startAutoTask(State.GO_TO_SCORE_POSITION, new TaskParams(), completionEvent);
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
                //Set elbow postion and drive to score position
                //wait for elbow only
                break;

            case SET_ELEVATOR_ARM:
                //Set elevator to score position
                //wait for elevator and PP
                break;

            case LOWER_ELBOW:
                //Lower elbow to clip specimen
                break;

            case SCORE_CHAMBER:
                //release specimen
                break;

            case RETRACT_ELEVATOR_ARM:
                //retract elevator, arm, and elbow "fire and forget"
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoScoreChamber