
package teamcode.autotasks;

import java.util.Locale;

import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.vision.Vision;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcVisionTargetInfo;


/**
 * This class implements auto-assist task to pick up a sample from ground.
 */

public class TaskAutoVisionPickupSample extends TrcAutoTask<TaskAutoVisionPickupSample.State>
{
    private static final String moduleName = TaskAutoVisionPickupSample.class.getSimpleName();

    public enum State
    {
        SET_START_POSES,
        FIND_SAMPLE,
        MOVE_TO_SAMPLE,
        PICKUP_SAMPLE,
        RETRACT_ELEVATOR,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        final Vision.SampleType sampleType;

        TaskParams(FtcAuto.Alliance alliance, Vision.SampleType sampleType)
        {
            this.alliance = alliance;
            this.sampleType = sampleType;
        }   //TaskParams

        public String toString()
        {
            return "alliance=" + alliance + ",sampleType=" + sampleType;
        }
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event1;
    private final TrcEvent event2;

    private String currOwner = null;
    private TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> sampleInfo = null;
    private TrcPose2D samplePose = null;
    private Double visionExpiredTime = null;


    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */

    public TaskAutoVisionPickupSample(String ownerName, Robot robot)
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
            FtcAuto.Alliance alliance, Vision.SampleType sampleType, TrcEvent completionEvent)
    {
        if (alliance == null)
        {
            // Caller is TeleOp, let's determine the alliance color by robot's location.
            alliance = robot.robotDrive.driveBase.getFieldPosition().y < 0.0?
                    FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        }

        TaskParams taskParams = new TaskParams(alliance, sampleType);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(State.SET_START_POSES, taskParams, completionEvent);
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
        robot.elbowElevator.cancel();
        robot.vision.setSampleVisionEnabled(Vision.SampleType.YellowSample,false);
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
            case SET_START_POSES:
                // Setup all systems to Vision Position
                robot.robotDrive.purePursuitDrive.start(
                        currOwner, event2, 0.0, false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        robot.adjustPoseByAlliance(RobotParams.Game.RED_NET_ZONE_VISION_PICKUP_POSE, taskParams.alliance, false));
                robot.vision.setSampleVisionEnabled(Vision.SampleType.YellowSample,true);
                robot.elbowElevator.setPosition(12.2, RobotParams.ElevatorParams.PICKUP_SAMPLE_POS, null,0,0.44, event1);
                robot.rotationalWrist.setPosition(null, 0, RobotParams.WristParamsRotational.PARALLEL_BASE_P0S, null, 0);
                robot.wristArm.setWristArmPosition(currOwner, 1.0, RobotParams.WristParamsVertical.SAMPLE_PICKUP_MODE_START, 0, null);
                robot.clawGrabber.open();
                sm.addEvent(event1);
                sm.addEvent(event2);
                sm.waitForEvents(State.FIND_SAMPLE, true);
                break;

            case FIND_SAMPLE:
                // Set Arm to Pickup Pos
                sampleInfo = robot.vision.getDetectedSample(taskParams.sampleType, 0.0, -1);
                if (sampleInfo != null)
                {
                    samplePose = robot.getDetectedSamplePose(sampleInfo, true);
                    // Vision found the sample.
                    String msg = String.format(
                            Locale.US, "%s is found at x %.1f, y %.1f, angle=%.1f, rotatedAngle=%.1f, rotatedAngleMaped=%.1f",
                            taskParams.sampleType, samplePose.x, samplePose.y, samplePose.angle,
                            sampleInfo.objRotatedRectAngle, robot.MapAngleRange(sampleInfo.objRotatedRectAngle, 90));
                    tracer.traceInfo(moduleName, msg);
                    sm.setState(State.MOVE_TO_SAMPLE);
                }
                else if (visionExpiredTime == null)
                {
                    // Vision doesn't find the sample, set a 1-second timeout and keep trying.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 0.75;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out and vision still not finding sample, giving up.
                    tracer.traceInfo(moduleName, "%s not found, we are done.", taskParams.sampleType);
                    sm.setState(State.DONE);
                }
                break;

            case MOVE_TO_SAMPLE:
                double elevatorLen = robot.getElevatorPosFromSamplePos(samplePose, sampleInfo.objRotatedRectAngle);
                robot.elbowElevator.setPosition(elevatorLen, null, null, event1);
                robot.robotDrive.purePursuitDrive.start(
                        currOwner, event2, 0.0, true,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        new TrcPose2D(samplePose.x, 0.0, 0.0));
                double wristAngle = robot.getRotationalWristAngleFromSamplePos(sampleInfo.objRotatedRectAngle);
                robot.rotationalWrist.setPosition(wristAngle);
                robot.wristArm.setWristArmPosition(currOwner, 0.48, robot.verticalWristPickupSamplePos(), 0, null);
                sm.addEvent(event1);
                sm.addEvent(event2);
                sm.waitForEvents(State.PICKUP_SAMPLE, true);
                break;

            case PICKUP_SAMPLE:
                //Fire and Forget
                robot.wristArm.setWristArmPickupSamplePos(currOwner,0,null);
                robot.clawGrabber.close(null,0.05,event1);
                sm.waitForSingleEvent(event1, State.RETRACT_ELEVATOR);
                break;

            case RETRACT_ELEVATOR:
                robot.wristArm.setWristArmPosition(currOwner, RobotParams.ArmParams.SAMPLE_PICKUP_MODE_START, RobotParams.WristParamsVertical.SAMPLE_PICKUP_MODE_START, 0, null);
                robot.elbowElevator.setPosition(RobotParams.ElevatorParams.PICKUP_SAMPLE_POS, null, null, null);
                sm.setState(State.DONE);
                break;

            default:
            case DONE:
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoPickupFromGround
