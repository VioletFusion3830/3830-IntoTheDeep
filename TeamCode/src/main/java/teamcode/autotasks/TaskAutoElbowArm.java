//package teamcode.autotasks;
//
//import teamcode.Robot;
//import trclib.robotcore.TrcAutoTask;
//import trclib.robotcore.TrcEvent;
//import trclib.robotcore.TrcOwnershipMgr;
//import trclib.robotcore.TrcRobot;
//import trclib.robotcore.TrcTaskMgr;
//
///**
// * This class implements auto-assist task.
// */
//public class TaskAutoELbowArm extends TrcAutoTask<TaskAutoELbowArm.State> {
//    private static final String moduleName = TaskAutoELbowArm.class.getSimpleName();
//
//    public enum State
//    {
//        SET_POSITION,
//        RETRACT_EXTENDER,
//        SET_ELBOW_ANGLE,
//        SET_EXTENDER_POSITION,
//        WAIT_FOR_COMPLETION,
//        DONE
//    }   //enum State
//
//    private static class TaskParams {
//        TaskParams() {
//        }   //TaskParams
//    }   //class TaskParams
//
//    private final String ownerName;
//    private final Robot robot;
//    private TrcEvent event;
//
//    private String currOwner = null;
//
//    /**
//     * Constructor: Create an instance of the object.
//     *
//     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
//     * @param robot     specifies the robot object that contains all the necessary subsystems.
//     */
//    public TaskAutoELbowArm(String ownerName, Robot robot) {
//        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
//        this.ownerName = ownerName;
//        this.robot = robot;
//        event = new TrcEvent(moduleName);
//    }   //TaskAuto
//
//    /**
//     * This method starts the auto-assist operation.
//     *
//     * @param completionEvent specifies the event to signal when done, can be null if none provided.
//     */
//    public void autoSetElbow(TrcEvent completionEvent) {
//        tracer.traceInfo(moduleName, "event=" + completionEvent);
//        startAutoTask(State.RETRACT_EXTENDER, new TaskParams(), completionEvent);
//    }   //autoAssist
//
//    //
//    // Implement TrcAutoTask abstract methods.
//    //
//
//    /**
//     * This method is called by the super class to acquire ownership of all subsystems involved in the auto-assist
//     * operation. This is typically done before starting an auto-assist operation.
//     *
//     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
//     * failed.
//     */
//    @Override
//    protected boolean acquireSubsystemsOwnership() {
//        boolean success = ownerName == null ||
//                (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName));
//
//        if (success) {
//            currOwner = ownerName;
//            tracer.traceInfo(moduleName, "Successfully acquired subsystem ownerships.");
//        } else {
//            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
//            tracer.traceWarn(
//                    moduleName,
//                    "Failed to acquire subsystem ownership (currOwner=" + currOwner +
//                            ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
//            releaseSubsystemsOwnership();
//        }
//
//        return success;
//    }   //acquireSubsystemsOwnership
//
//    /**
//     * This method is called by the super class to release ownership of all subsystems involved in the auto-assist
//     * operation. This is typically done if the auto-assist operation is completed or canceled.
//     */
//    @Override
//    protected void releaseSubsystemsOwnership() {
//        if (ownerName != null) {
//            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
//            tracer.traceInfo(
//                    moduleName,
//                    "Releasing subsystem ownership (currOwner=" + currOwner +
//                            ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
//            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
//            currOwner = null;
//        }
//    }   //releaseSubsystemsOwnership
//
//    /**
//     * This method is called by the super class to stop all the subsystems.
//     */
//    @Override
//    protected void stopSubsystems() {
//        tracer.traceInfo(moduleName, "Stopping subsystems.");
//        robot.robotDrive.cancel(currOwner);
//    }   //stopSubsystems
//
//    /**
//     * This methods is called periodically to run the auto-assist task.
//     *
//     * @param params           specifies the task parameters.
//     * @param state            specifies the current state of the task.
//     * @param taskType         specifies the type of task being run.
//     * @param runMode          specifies the competition mode (e.g. Autonomous, TeleOp, Test).
//     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
//     *                         false if running the fast loop on the main robot thread.
//     */
//    @Override
//    protected void runTaskState(
//            Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop) {
//        TaskParams taskParams = (TaskParams) params;
//
//        switch (state) {
//            case RETRACT_EXTENDER:
//                if (taskParams.elbowAngle != null &&
//                        Math.abs(extender.getPosition() - Extender.Params.MIN_POS) >
//                                Extender.Params.POS_PID_TOLERANCE) {
//                    // We are setting the elbow angle and the extender is extended, retract it first.
//                    extender.setPosition(
//                            currOwner, 0.0, Extender.Params.MIN_POS, true, Extender.Params.POWER_LIMIT, extenderEvent, 0.0);
//                    sm.waitForSingleEvent(extenderEvent, State.SET_ELBOW_ANGLE);
//                } else {
//                    // Either we are not setting elbow angle or the extender is already retracted, skip this state.
//                    sm.setState(State.SET_ELBOW_ANGLE);
//                }
//                break;
//
//            case SET_ELBOW_ANGLE:
//                if (taskParams.elbowAngle != null) {
//                    // We are setting elbow angle, go do it.
//                    elbow.setPosition(
//                            currOwner, 0.0, taskParams.elbowAngle, true, Elbow.Params.POWER_LIMIT, elbowEvent, 4.0);
//                    if (safeSequence) {
//                        sm.waitForSingleEvent(elbowEvent, State.SET_EXTENDER_POSITION);
//                    } else {
//                        // Not performing safe sequence, so don't wait.
//                        sm.setState(State.SET_EXTENDER_POSITION);
//                    }
//                } else {
//                    // Caller did not provide elbow angle, skip this state.
//                    elbowEvent.signal();
//                    sm.setState(State.SET_EXTENDER_POSITION);
//                }
//                break;
//
//            case SET_EXTENDER_POSITION:
//                if (taskParams.extenderPosition != null) {
//                    // We are setting extender position, go do it.
//                    extender.setPosition(
//                            currOwner, 0.0, taskParams.extenderPosition, true, Extender.Params.POWER_LIMIT, extenderEvent,
//                            4.0);
//                    if (safeSequence) {
//                        sm.waitForSingleEvent(extenderEvent, State.WAIT_FOR_COMPLETION);
//                    } else {
//                        // Not performing safe sequence, so don't wait.
//                        sm.setState(State.WAIT_FOR_COMPLETION);
//                    }
//                } else {
//                    // We are not setting extender position, we are done.
//                    extenderEvent.signal();
//                    sm.setState(State.WAIT_FOR_COMPLETION);
//                }
//                break;
//
//            case WAIT_FOR_COMPLETION:
//                if (safeSequence) {
//                    // If we performed safe sequence and came here, it means both events are already signaled.
//                    sm.setState(State.DONE);
//                } else {
//                    sm.addEvent(elbowEvent);
//                    sm.addEvent(extenderEvent);
//                    // Don't clear the events.
//                    sm.waitForEvents(State.DONE, false, true);
//                }
//                break;
//
//            default:
//            case DONE:
//                // Stop task.
//                stopAutoTask(true);
//                break;
//        }
//    }
//}