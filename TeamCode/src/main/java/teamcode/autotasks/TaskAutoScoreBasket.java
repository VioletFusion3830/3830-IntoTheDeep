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
 * This class implements auto-assist task.
 */
public class TaskAutoScoreBasket extends TrcAutoTask<TaskAutoScoreBasket.State>
{
    private static final String moduleName = TaskAutoScoreBasket.class.getSimpleName();

    public enum State
    {
        GO_TO_SCORE_POSITION,
        SET_ARM,
        SCORE_BASKET,
        RETRACT_ELEVATOR_ARM,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        final TrcPose2D[] scorePose;
        double elevatorDelay;

        TaskParams(FtcAuto.Alliance alliance, TrcPose2D[] scorePose, double elevatorDelay )
        {
            this.alliance = alliance;
            this.scorePose = scorePose;
            this.elevatorDelay = elevatorDelay;
        }   //TaskParams

        public String toString()
        {
            return "alliance=" + alliance + "isVisionCycle=" + elevatorDelay;
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
    public TaskAutoScoreBasket(String ownerName, Robot robot)
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
    public void autoScoreBasket(FtcAuto.Alliance alliance, TrcPose2D[] scorePose, double elevatorDelay, TrcEvent completionEvent)
    {
        if (alliance == null)
        {
            // Caller is TeleOp, let's determine the alliance color by robot's location.
            alliance = robot.robotDrive.driveBase.getFieldPosition().y < 0.0?
                    FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        }

        TaskParams taskParams = new TaskParams(alliance, scorePose, elevatorDelay);
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
        robot.verticalWrist.cancel();
        robot.arm.cancel();
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
                robot.robotDrive.purePursuitDrive.start(
                        currOwner, event2, 0.0, false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        robot.adjustPoseByAlliance(taskParams.scorePose, taskParams.alliance, false));
                robot.elbowElevator.setPosition(null,RobotParams.ElbowParams.BASKET_SCORE_POS,RobotParams.ElevatorParams.HIGH_BASKET_SCORE_POS,taskParams.elevatorDelay,0,event1);
                robot.rotationalWrist.setPosition(null,0,RobotParams.WristParamsRotational.PERPENDICULAR_POS,null,0);
                robot.wristArm.setWristArmPosition(currOwner,0.56,0.48 ,0,null);
                sm.waitForSingleEvent(event1, State.SET_ARM);
                break;

            case SET_ARM:
                robot.wristArm.setWristArmBasketScorePos(currOwner,0.13, event1);
                event1.clear();
                sm.addEvent(event1);
                sm.addEvent(event2);
                sm.waitForEvents(State.SCORE_BASKET,false,true);
                break;

            case SCORE_BASKET:
                event1.clear();
                robot.clawGrabber.open(null,event1);
                sm.waitForSingleEvent(event1,State.RETRACT_ELEVATOR_ARM,false,0.2);
                break;

            case RETRACT_ELEVATOR_ARM:
                //retract elevator, elbow, and arm "fire and forget"
                robot.wristArm.setWristArmPosition(currOwner,RobotParams.ArmParams.SAMPLE_PICKUP_MODE_START,RobotParams.WristParamsVertical.SAMPLE_PICKUP_MODE_START,0,null);
                robot.elbowElevator.setPosition(true,RobotParams.ElevatorParams.PICKUP_SAMPLE_POS,RobotParams.ElbowParams.PICKUP_SAMPLE_POS,null, null);
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