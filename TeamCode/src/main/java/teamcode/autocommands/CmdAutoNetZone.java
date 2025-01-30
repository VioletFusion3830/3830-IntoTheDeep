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
public class CmdAutoNetZone implements TrcRobot.RobotCommand
{
    private final String moduleName = getClass().getSimpleName();

    private enum State
    {
        START,
        DO_DELAY,
        SCORE_PRELOAD,
        PICKUP_SAMPLE_MARK_1,
        SCORE_SAMPLE_BASKET_1,
        PICKUP_SAMPLE_MARK_2,
        SCORE_SAMPLE_BASKET_2,
        PICKUP_SAMPLE_MARK_3,
        SCORE_SAMPLE_BASKET_3,
        GO_PARK,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int scoreSampleCount = 0;
    private int maxSampleCount = 3;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdAutoNetZone(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAuto

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
        robot.scoreBasketTask.cancel();
        robot.autoPickupSample.cancel();
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
                        sm.waitForSingleEvent(event, State.SCORE_PRELOAD);
                    }
                    else
                    {
                        sm.setState(State.SCORE_PRELOAD);
                    }
                    break;

                case SCORE_PRELOAD:
                    // Score the preloaded sample or specimen.
                    if (autoChoices.preloadType == Robot.GamePieceType.SPECIMEN)
                    {
                        robot.scoreChamberTask.autoScoreChamber(null,false, event);
                    }
                    else
                    {
                        robot.scoreBasketTask.autoScoreBasket(autoChoices.alliance,RobotParams.Game.RED_BASKET_SCORE_POSE ,event);
                    }
                    sm.waitForSingleEvent(event, State.PICKUP_SAMPLE_MARK_1);
                    break;

                case PICKUP_SAMPLE_MARK_1:
                    // Pickup first floor sample.
                    robot.autoPickupSample.autoPickupSample(autoChoices.alliance,new TrcPose2D(-47,-45,0), RobotParams.WristParamsRotational.PARALLEL_BASE_P0S,event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case SCORE_SAMPLE_BASKET_1:
                    // Score first floor sample into the sample in the basket.
                    robot.scoreBasketTask.autoScoreBasket(autoChoices.alliance, RobotParams.Game.RED_BASKET_SCORE_POSE, event);
                    sm.waitForSingleEvent(event, State.PICKUP_SAMPLE_MARK_2);
                    break;

                case PICKUP_SAMPLE_MARK_2:
                    // Pickup second floor sample.
                    robot.autoPickupSample.autoPickupSample(autoChoices.alliance, new TrcPose2D(-55,-20,180), RobotParams.WristParamsRotational.PARALLEL_SECONDARY_POS,event);
                    sm.waitForSingleEvent(event, State.SCORE_SAMPLE_BASKET_2);
                    break;

                case SCORE_SAMPLE_BASKET_2:
                    // Score second floor sample into the sample in the basket.
                    robot.scoreBasketTask.autoScoreBasket(autoChoices.alliance, RobotParams.Game.RED_BASKET_SCORE_POSE, event);
                    sm.waitForSingleEvent(event, State.PICKUP_SAMPLE_MARK_3);
                    break;

                case PICKUP_SAMPLE_MARK_3:
                    // Pickup third floor sample.
                    robot.autoPickupSample.autoPickupSample(autoChoices.alliance, new TrcPose2D(-55,-20,145), 0.45,event);
                    sm.waitForSingleEvent(event, State.SCORE_SAMPLE_BASKET_3);
                    break;

                case SCORE_SAMPLE_BASKET_3:
                    // Score third floor sample into the sample in the basket.
                    robot.scoreBasketTask.autoScoreBasket(autoChoices.alliance, RobotParams.Game.RED_BASKET_SCORE_POSE, event);
                    sm.waitForSingleEvent(event, State.PARK);
                    break;

                case PARK:
                    if (autoChoices.parkPos == FtcAuto.ParkOption.PARK)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                                event, 0.0, false,
                                robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                                robot.adjustPoseByAlliance(
                                        RobotParams.Game.RED_ASCENT_ZONE_PARK_POSE, autoChoices.alliance, false));
                        robot.autoHang.autoClimbLevel1(null);
                        sm.waitForSingleEvent(event, State.DONE);
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