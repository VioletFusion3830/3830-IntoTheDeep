
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

public class TaskAutoSweepSamples extends TrcAutoTask<TaskAutoSweepSamples.State>
{
    private static final String moduleName = TaskAutoSweepSamples.class.getSimpleName();

    public enum State
    {
        GO_TO_START_POSITION,
        LOWER_ARM,
        SWEEP_SAMPLE,
        REST_FOR_SAMPLE_2,
        REST_FOR_SAMPLE_3,
        SET_SUBSYSTEMS,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;

        TaskParams(FtcAuto.Alliance alliance)
        {
            this.alliance = alliance;
        }   //TaskParams

        public String toString()
        {
            return "alliance=" + alliance;
        }
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event1;
    private final TrcEvent event2;

    private String currOwner = null;
    private int samplesSweeped = 0;


    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */

    public TaskAutoSweepSamples(String ownerName, Robot robot)
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

    public void sweepSpikeMarkSamples(
            FtcAuto.Alliance alliance, TrcEvent completionEvent)
    {
        TaskParams taskParams = new TaskParams(alliance);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(State.GO_TO_START_POSITION, taskParams, completionEvent);
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
            case GO_TO_START_POSITION:
                // Setup all systems to pickup position.
                robot.robotDrive.purePursuitDrive.getTurnPidCtrl().setNoOscillation(true);
                TrcPose2D[] RED_OBSERVATION_ZONE_SWEEP_SAMPLE_1 = {
                        new TrcPose2D(8, -54, 40),
                        new TrcPose2D(29, -40, 40)
                };
                robot.robotDrive.purePursuitDrive.start(
                        currOwner, event2, 0.0, false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        robot.adjustPoseByAlliance(RED_OBSERVATION_ZONE_SWEEP_SAMPLE_1, taskParams.alliance, false));
                robot.elbowElevator.setPosition(RobotParams.ElbowParams.PICKUP_SPECIMEN_POS,26.0,0.5,0, event1);
                robot.wristArm.setWristArmPosition(currOwner,0.8,0.08,0.67,0,null);
                sm.addEvent(event1);
                sm.addEvent(event2);
                sm.waitForEvents(State.SWEEP_SAMPLE, true);
                break;

            case LOWER_ARM:
                robot.wristArm.setWristArmPosition(currOwner,0.08,0.67,0.15,event1);
                sm.waitForSingleEvent(event1, State.SWEEP_SAMPLE);
                break;

            case SWEEP_SAMPLE:
                robot.robotDrive.purePursuitDrive.getTurnPidCtrl().setNoOscillation(true);
                samplesSweeped++;
                robot.robotDrive.purePursuitDrive.start(
                        currOwner, event1, 0.0, false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        robot.adjustPoseByAlliance(new TrcPose2D(35, -50, 120), taskParams.alliance));
                robot.elbowElevator.setPosition(null, 20.0, null);
                if(samplesSweeped == 1) sm.waitForSingleEvent(event1, State.REST_FOR_SAMPLE_2);
                else if(samplesSweeped == 2) sm.waitForSingleEvent(event1, State.REST_FOR_SAMPLE_3);
                else sm.waitForSingleEvent(event1, State.DONE);
                break;

            case REST_FOR_SAMPLE_2:
                robot.robotDrive.purePursuitDrive.start(
                        currOwner, event1, 0.0, false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        robot.adjustPoseByAlliance(new TrcPose2D(39,-40,40), taskParams.alliance));
                robot.elbowElevator.setPosition(null,27.0,0.2,0, null);
                robot.wristArm.setWristArmPosition(currOwner,0.0,0.3,0.68,0,null);
                sm.waitForSingleEvent(event1, State.LOWER_ARM);
                break;

            case REST_FOR_SAMPLE_3:
                robot.robotDrive.purePursuitDrive.start(
                        currOwner, event1, 0.0, false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        robot.adjustPoseByAlliance(new TrcPose2D(43,-40,40), taskParams.alliance));
                robot.elbowElevator.setPosition(null,26.0,0.2,0, null);
                robot.wristArm.setWristArmPosition(currOwner,0.0,0.3,0.68,0,null);
                sm.waitForSingleEvent(event1, State.LOWER_ARM);
                break;

            case SET_SUBSYSTEMS:
                robot.elbowElevator.setPosition(null, RobotParams.ElevatorParams.PICKUP_SPECIMEN_POS, null);
                robot.wristArm.setWristArmPosition(currOwner,0.0,0.2,0.68,0,null);
                robot.robotDrive.purePursuitDrive.getTurnPidCtrl().setNoOscillation(false);
                sm.setState(State.DONE);
                break;

            default:
            case DONE:
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoPickupFromGround
