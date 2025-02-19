package teamcode.autocommands;

import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdAutoObservationZone implements TrcRobot.RobotCommand
{
    private final String moduleName = getClass().getSimpleName();

    private enum State
    {
        START,
        DO_DELAY,
        SET_POSITIONS_TO_SCORE_PRELOAD,
        SCORE_PRELOAD,
        MOVE_SAMPLES,
        PICKUP_SPECIMEN,
        SCORE_SPECIMEN,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent event2;
    private final TrcStateMachine<State> sm;
    private int scoreSpecimenCount = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdAutoObservationZone(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        event2 = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAutoObservationZone

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        timer.cancel();
        robot.scoreChamberTask.cancel();
        robot.pickupSpecimenTask.cancel();
        robot.elbowElevator.cancel();
        robot.arm.cancel();
        robot.verticalWrist.cancel();
        robot.clawGrabber.cancel();
        robot.rotationalWrist.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + sm.getNextState() + ")...");
        }
        else
        {
            robot.dashboard.displayPrintf(8, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case START:
                    // Set robot location according to auto choices.
                    robot.setRobotStartPosition(autoChoices);
                    //
                    // Intentionally fall to next state.
                    //
                case DO_DELAY:
                    // Do delay if there is one.
                    if (autoChoices.delay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + autoChoices.delay + "s.");
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.SET_POSITIONS_TO_SCORE_PRELOAD);
                    }
                    else
                    {
                        sm.setState(State.SET_POSITIONS_TO_SCORE_PRELOAD);
                    }
                    break;

                case SET_POSITIONS_TO_SCORE_PRELOAD:
                    robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                            robot.adjustPoseByAlliance(
                                    RobotParams.Game.RED_OBSERVATION_FORWARD_CHAMBER_SCORE_POSE, autoChoices.alliance, false));
                    robot.elbowElevator.setPosition(40.0,27.0,event2);
                    robot.wristArm.setWristArmPosition(0.5,0.25);
                    sm.addEvent(event);
                    sm.addEvent(event2);
                    sm.waitForEvents(State.SCORE_PRELOAD,true);
                    break;

                case SCORE_PRELOAD:
                    // Score the preloaded specimen.
                    robot.clawGrabber.open(event);
                    robot.elbowElevator.setPosition(39.0,13.0,null);
                    sm.waitForSingleEvent(event, State.MOVE_SAMPLES);
                    break;

                case MOVE_SAMPLES:
                    // Herd three samples to the observation zone to be converted to specimens.
                    robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                            robot.adjustPoseByAlliance(
                                    RobotParams.Game.RED_OBSERVATION_ZONE_SAMPLE_MOVE_PATH, autoChoices.alliance, false));
                    //Set Arm up so it does not hit the wall
                    robot.wristArm.setWristArmPosition(null, 0, 0.8, RobotParams.WristParamsVertical.PICKUP_SPECIMEN_POS,0,null);
                    robot.rotationalWrist.setPosition(null,0,RobotParams.WristParamsRotational.PARALLEL_BASE_P0S,null,0);
                    //Set Arm to pick sample off wall
                    robot.wristArm.setWristArmPosition(null, 6, RobotParams.ArmParams.PICKUP_SPECIMEN_POS, RobotParams.WristParamsVertical.PICKUP_SPECIMEN_POS,0,null);
                    robot.elbowElevator.setPosition(RobotParams.ElbowParams.PICKUP_SPECIMEN_POS,13.0, null);
                    sm.waitForSingleEvent(event, State.PICKUP_SPECIMEN);
                    break;

                case PICKUP_SPECIMEN:
                    // Pick up a specimen from the wall.
                    if (scoreSpecimenCount < 4)
                    {
                        scoreSpecimenCount++;
                        robot.pickupSpecimenTask.autoPickupSpecimen(autoChoices.alliance, scoreSpecimenCount == 1, event);
                        sm.waitForSingleEvent(event,State.SCORE_SPECIMEN);
                    }
                    else sm.setState(State.PARK);
                    break;

                case SCORE_SPECIMEN:
                    // Score the specimen.
                    TrcPose2D[] scorePose;
                    if(scoreSpecimenCount == 1)
                    {
                        scorePose = new TrcPose2D[]{
                                new TrcPose2D(11, -37, 180),
                                new TrcPose2D(10, -33.5, 180),
                                new TrcPose2D(7, -33.5, 180)
                        };
                    }
                    else
                    {
                        scorePose = new TrcPose2D[]{
                                new TrcPose2D(9, -35, 180),
                                new TrcPose2D(7, -33.5, 180)
                        };
                    }
                    robot.scoreChamberTask.autoScoreChamber(autoChoices.alliance, scorePose,true, event);
                    sm.waitForSingleEvent(event, State.PICKUP_SPECIMEN);
                    break;

                case PARK:
                    // Park at the observation zone.
                    if (autoChoices.parkPos == FtcAuto.ParkOption.PARK)
                    {
                        robot.elbowElevator.setPosition(true,13.0, RobotParams.ElbowParams.MIN_POS, null, event);
                        robot.wristArm.setWristArmPickupSpecimenPos();
                        robot.robotDrive.purePursuitDrive.start(
                                event2, 0.0, false,
                                robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                                robot.adjustPoseByAlliance(RobotParams.Game.RED_OBSERVATION_ZONE_PARK_POSE, autoChoices.alliance, false));
                        sm.addEvent(event);
                        sm.addEvent(event2);
                        sm.waitForEvents(State.DONE, true);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                default:
                case DONE:
                    // We are done.
                    cancel();
                    break;
            }

            robot.globalTracer.tracePostStateInfo(
                    sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto