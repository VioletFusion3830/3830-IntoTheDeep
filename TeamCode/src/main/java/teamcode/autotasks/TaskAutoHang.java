package teamcode.autotasks;

import teamcode.FtcDashboard;
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
public class TaskAutoHang extends TrcAutoTask<TaskAutoHang.State>
{
    private static final String moduleName = TaskAutoHang.class.getSimpleName();

    public enum State
    {
        LEVEL1_ASCENT,
        LEVEL2_START,
        CLIP,
        LEVEL2_ASCENT,
        LEVEL2_FINISH,
        LEVEL3_START,
        DONE
    }   //enum State

    private static class TaskParams
    {
        TaskParams()
        {
        }   //TaskParams
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private TrcEvent event;
    private TrcEvent event2;
    private String currOwner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoHang(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        event = new TrcEvent(moduleName);
        event2 = new TrcEvent(moduleName);
    }   //TaskAutoClimb


    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoClimbLevel1(TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "event=" + completionEvent);
        startAutoTask(State.LEVEL1_ASCENT, null, completionEvent);
    }   //autoClimbLevel1

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoClimbLevel2(TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "event=" + completionEvent);
        startAutoTask(State.LEVEL2_START, null, completionEvent);
    }   //autoClimbLevel2

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoClimbLevel3(TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "event=" + completionEvent);
        startAutoTask(State.LEVEL3_START, null, completionEvent);
    }   //autoClimbLevel3

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
        robot.clawServo.cancel();
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
            case LEVEL1_ASCENT:
                robot.elbowElevator.setPosition(RobotParams.ElbowParams.LEVEL1_ASCENT_POS,RobotParams.ElevatorParams.LEVEL1_ASCENT_POS,event);
                robot.wristArm.setWristArmPosition(currOwner,.5,.5,0,null);
                sm.waitForSingleEvent(event,State.DONE);
                break;

            case LEVEL2_START:
                robot.elevator.setPosition(currOwner,0.5,RobotParams.ElevatorParams.LEVEL2_ASCENT_START_POS,true,RobotParams.ElevatorParams.POWER_LIMIT,event,2);
                robot.elbow.setPosition(currOwner,0,95,true,RobotParams.ElevatorParams.POWER_LIMIT,null,3);
                robot.arm.setPosition(null,0,.8,null,3);
                robot.verticalWrist.setPosition(null,0,.17,null,2);
                sm.addEvent(event);
                sm.waitForEvents(State.CLIP);
                break;

            case CLIP:
                robot.elbow.setPosition(currOwner,0,RobotParams.ElbowParams.LEVEL2_ASCENT_START_POS,true,RobotParams.ElbowParams.POWER_LIMIT,event,2);
                sm.addEvent(event);
                sm.waitForEvents(State.LEVEL2_ASCENT);
                break;

            case LEVEL2_ASCENT:
                robot.elevator.setPidStallDetectionEnabled(false);
                robot.elevator.setStallProtection(0.0, 0.0, 0.0, 0.0);
                //robot.elbow.setPositionPidParameters(FtcDashboard.TunePID.tunePidCoeff, RobotParams.ElbowParams.PID_TOLERANCE);
                robot.elevator.setPositionPidParameters(1,0,0,0,0, RobotParams.ElevatorParams.POS_PID_TOLERANCE);
                robot.elevator.setPosition(currOwner,0,RobotParams.ElevatorParams.LEVEL2_ASCENT_POS,true,RobotParams.ElevatorParams.POWER_LIMIT,event2,2);
                robot.elbow.setPosition(currOwner,0.5,RobotParams.ElbowParams.LEVEL2_ASCENT_POS,true,RobotParams.ElbowParams.POWER_LIMIT,event,2);
                sm.waitForSingleEvent(event2,State.LEVEL2_FINISH);
                break;

            case LEVEL2_FINISH:
                sm.waitForSingleEvent(event,State.DONE);

            case LEVEL3_START:
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