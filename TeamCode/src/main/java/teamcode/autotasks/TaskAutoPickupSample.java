
package teamcode.autotasks;

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
 * This class implements auto-assist task to pick up a sample from ground.
 */

public class TaskAutoPickupSample extends TrcAutoTask<TaskAutoPickupSample.State>
{
    private static final String moduleName = TaskAutoPickupSample.class.getSimpleName();

    public enum State
    {
        GO_TO_POSITION,
        PICKUP_SAMPLE,
        RAISE_ELEVATOR,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        final TrcPose2D scorePose;
        final Double wirstRotationalPos;

        TaskParams(FtcAuto.Alliance alliance, TrcPose2D scorePose, Double wirstRotationalPos)
        {
            this.alliance = alliance;
            this.scorePose = scorePose;
            this.wirstRotationalPos = wirstRotationalPos;
        }   //TaskParams

        public String toString()
        {
            return "alliance=" + alliance + ", scorePose=" + scorePose + ", wirstRotationalPos=" + wirstRotationalPos;
        }
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

    public TaskAutoPickupSample(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        this.event1 = new TrcEvent(moduleName + ".event1");
        this.event2 = new TrcEvent(moduleName + ".event2");
    }   //TaskAutoPickupFromGround

/**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */

public void autoPickupSample(
        FtcAuto.Alliance alliance, TrcPose2D scorePose, Double wirstRotationalPos, TrcEvent completionEvent)
{
    if (alliance == null)
    {
        // Caller is TeleOp, let's determine the alliance color by robot's location.
        alliance = robot.robotDrive.driveBase.getFieldPosition().y < 0.0?
                FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
    }
    if(scorePose == null)
    {
        scorePose = RobotParams.Game.RED_NET_ZONE_SPIKEMARK_PICKUP.clone();
    }
    if(wirstRotationalPos == null)
    {
        wirstRotationalPos = RobotParams.WristParamsRotational.PARALLEL_BASE_P0S;
    }

    TaskAutoPickupSample.TaskParams taskParams = new TaskAutoPickupSample.TaskParams(alliance, scorePose, wirstRotationalPos);
    tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
    startAutoTask(State.GO_TO_POSITION, taskParams, completionEvent);
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
        robot.sampleTeleOpMacros.cancel();
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
        State nextState;

        switch (state)
        {
            case GO_TO_POSITION:
                // Setup all systems to pickup position.
                robot.robotDrive.purePursuitDrive.start(
                        currOwner, event2, 0.0, false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        robot.adjustPoseByAlliance(taskParams.scorePose, taskParams.alliance));
                robot.elbowElevator.setPosition(true, RobotParams.ElevatorParams.PICKUP_SAMPLE_POS, RobotParams.ElbowParams.PICKUP_SPECIMEN_POS,null, event1);
                robot.rotationalWrist.setPosition(null,0,taskParams.wirstRotationalPos,null,0);
                robot.wristArm.setWristArmPickupSamplePos();
                sm.addEvent(event1);
                sm.addEvent(event2);
                sm.waitForEvents(State.PICKUP_SAMPLE, true);
                break;

            case PICKUP_SAMPLE:
                // Pickup the sample.
                robot.sampleTeleOpMacros.autoPickSample(event1);
                sm.waitForSingleEvent(event1, State.RAISE_ELEVATOR);
                break;

            case RAISE_ELEVATOR:
                robot.elbowElevator.setPosition(true,RobotParams.ElbowParams.HIGH_CHAMBER_SCORE_POS,RobotParams.ElevatorParams.HIGH_BASKET_SCORE_POS,null);
                break;

            default:
            case DONE:
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoPickupFromGround
