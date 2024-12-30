/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

import ftclib.drivebase.FtcSwerveDrive;
import ftclib.driverio.FtcGamepad;
import ftclib.robotcore.FtcOpMode;
import teamcode.subsystems.Claw;
import trclib.drivebase.TrcDriveBase;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.timer.TrcTimer;

/**
 * This class contains the TeleOp Mode program.
 */
@TeleOp(name="FtcTeleOp", group="Ftcxxxx")
public class FtcTeleOp extends FtcOpMode
{
    private final String moduleName = getClass().getSimpleName();

    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    private double drivePowerScale;
    private double turnPowerScale;
    private boolean driverAltFunc = false;
    private boolean operatorAltFunc = false;
    private boolean relocalizing = false;
    private double elevatorPrevPower = 0.0, armPrevPos = 0.0, elbowPrevPower = 0.0, rotationalWristPrevPos = 0.0;
    private boolean isSampleTypeRedAlliance = false;
    private boolean isWristRotatorMiddle = false;
    private Double elevatorLimit = null;
    public static Claw.SamplePickupType SamplePickupType = Claw.SamplePickupType.anySample;
    public static boolean isSamplePickupMode = true, isClawGrabbing = false, is45Left = false;
    private ElapsedTime runtime;

    private TrcPose2D robotFieldPose = null;
    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void robotInit()
    {
        //
        // Create and initialize robot object.
        //
        robot = new Robot(TrcRobot.getRunMode());
        drivePowerScale = RobotParams.Robot.DRIVE_NORMAL_SCALE;
        turnPowerScale = RobotParams.Robot.TURN_NORMAL_SCALE;

        //
        // Open trace log.
        //
        if (RobotParams.Preferences.useTraceLog)
        {
            String filePrefix = Robot.matchInfo != null?
                String.format(Locale.US, "%s%02d_TeleOp", Robot.matchInfo.matchType, Robot.matchInfo.matchNumber):
                "Unknown_TeleOp";
            TrcDbgTrace.openTraceLog(RobotParams.Robot.LOG_FOLDER_PATH, filePrefix);
        }
        //
        // Create and initialize Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1);
        driverGamepad.setButtonEventHandler(this::driverButtonEvent);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2);
        operatorGamepad.setButtonEventHandler(this::operatorButtonEvent);
        driverGamepad.setLeftStickInverted(false, true);
        operatorGamepad.setRightStickInverted(false, true);
        operatorGamepad.setLeftStickInverted(false,true);
        setDriveOrientation(RobotParams.Robot.DRIVE_ORIENTATION);
        runtime = new ElapsedTime();


//        Set<Thread> threadSet = Thread.getAllStackTraces().keySet();


    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(
            moduleName, "***** Starting TeleOp: " + TrcTimer.getCurrentTimeString() + " *****");
        robot.dashboard.clearDisplay();
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);
        //
        // Enable AprilTag vision for re-localization.
        //
        if (robot.vision != null && robot.vision.aprilTagVision != null)
        {
            robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision.");
            robot.vision.setAprilTagVisionEnabled(true);
        }
        robot.clawGrabber.close();
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);
        printPerformanceMetrics();
        robot.globalTracer.traceInfo(
            moduleName, "***** Stopping TeleOp: " + TrcTimer.getCurrentTimeString() + " *****");

        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.closeTraceLog();
        }
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            //
            // DriveBase subsystem.
            //
            if (robot.robotDrive != null) {
                // We are trying to re-localize the robot and vision hasn't seen AprilTag yet.
                if (relocalizing) {
                    if (robotFieldPose == null) {
                        robotFieldPose = robot.vision.getRobotFieldPose();
                    }
                } else {
                    double[] inputs = driverGamepad.getDriveInputs(
                            RobotParams.Robot.DRIVE_MODE, true, drivePowerScale, turnPowerScale);

                    if (robot.robotDrive.driveBase.supportsHolonomicDrive()) {
                        robot.robotDrive.driveBase.holonomicDrive(
                                null, inputs[0], inputs[1], inputs[2], robot.robotDrive.driveBase.getDriveGyroAngle());
                    } else {
                        robot.robotDrive.driveBase.arcadeDrive(inputs[1], inputs[2]);
                    }
                    robot.dashboard.displayPrintf(
                            1, "RobotDrive: Power=(%.2f,y=%.2f,rot=%.2f),Mode:%s",
                            inputs[0], inputs[1], inputs[2], robot.robotDrive.driveBase.getDriveOrientation());
                }
                double slowDriveTriggered = driverGamepad.getRightTrigger() * RobotParams.Robot.DRIVE_NORMAL_SCALE;
                double scaleFactor = Math.max(Math.abs(1 - slowDriveTriggered), 0.3);
                drivePowerScale = (Math.atan(5 * RobotParams.Robot.DRIVE_NORMAL_SCALE) / Math.atan(5)) * scaleFactor;
                turnPowerScale = (Math.atan(5 * RobotParams.Robot.TURN_NORMAL_SCALE) / Math.atan(5)) * scaleFactor;
            }
            //
            // Other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                if (robot.elbow != null && robot.elevator != null)
                {
                    double elbowPower = operatorGamepad.getRightStickY(true) * RobotParams.ElbowParams.POWER_LIMIT;
                    double elbowPos = robot.elbow.getPosition();

                    if (robot.elevator != null && elbowPos < RobotParams.ElbowParams.RESTRICTED_POS_THRESHOLD)
                    {
                        double elbowPosRadians = Math.toRadians(elbowPos);
                        elevatorLimit = RobotParams.ElevatorParams.MAX_POS - (Math.max(Math.cos(elbowPosRadians) * (isSamplePickupMode ? RobotParams.ElevatorParams.HORIZONTAL_LIMIT: RobotParams.ElevatorParams.HORIZONTAL_LIMIT), 0));
                        if (robot.elevator.getPosition() > elevatorLimit)
                        {
                            robot.globalTracer.traceInfo(null,"elevatorLimit:" + elevatorLimit);
                            //robot.elevator.setPosition(elevatorLimit);
                        }
                    }
                    else
                    {
                        elevatorLimit = RobotParams.ElevatorParams.MAX_POS;
                    }

                    if (elbowPower != elbowPrevPower)
                    {
                        if (operatorAltFunc)
                        {
                            robot.elbow.setPower(elbowPower);
                        }
                        else
                        {
                            robot.elbow.setPidPower(elbowPower, RobotParams.ElbowParams.MIN_POS, RobotParams.ElbowParams.MAX_POS, true);
                        }
                        elbowPrevPower = elbowPower;
                    }

                    if (robot.elevator != null)
                    {
                        double elevatorPower = operatorGamepad.getLeftStickY(true) * RobotParams.ElevatorParams.POWER_LIMIT;
                        if (elevatorPower != elevatorPrevPower)
                        {
                            if (operatorAltFunc)
                            {
                                robot.elevator.setPower(elevatorPower);
                            }
                            else
                            {
                                robot.elevator.setPidPower(elevatorPower, RobotParams.ElevatorParams.MIN_POS, 40, true);//elevatorLimit
                            }
                        }
                        elevatorPrevPower = elevatorPower;
                    }
                }
                if (robot.rotationalWrist != null)
                {
                    double rotationalWristIncrement = (-operatorGamepad.getLeftTrigger() + operatorGamepad.getRightTrigger()) * RobotParams.WristParamsRotational.ANALOG_INCREMENT;
                    double rotationalWristPos = robot.rotationalWrist.getPosition() + rotationalWristIncrement;

                    if (rotationalWristPos != rotationalWristPrevPos)
                    {
                        robot.rotationalWrist.setPosition(rotationalWristPos);
                    }
                    rotationalWristPrevPos = rotationalWristPos;

                }
                if(isClawGrabbing && runtime.seconds() > 0.33)
                {
                    isClawGrabbing = false;
                    robot.wristArm.setWristArmPosition(robot.armElevatorScaling(),robot.vWristElevatorScaling());
                }
                if(isSamplePickupMode && !isClawGrabbing && robot.wristArm != null)
                {
                    double armPos = robot.armElevatorScaling();

                    if (armPos != armPrevPos)
                    {
                            robot.wristArm.setWristArmPosition(armPos,robot.vWristElevatorScaling());
                    }
                    armPrevPos = armPos;
                }
            }
            // Display subsystem status.
            if (RobotParams.Preferences.doStatusUpdate)
            {
            robot.updateStatus(2);
            }
        }
    }   //periodic

    /**
     * This method sets the drive orientation mode and updates the LED to indicate so.
     *
     * @param orientation specifies the drive orientation (FIELD, ROBOT, INVERTED).
     */
    public void setDriveOrientation(TrcDriveBase.DriveOrientation orientation)
    {
        if (robot.robotDrive != null)
        {
            robot.globalTracer.traceInfo(moduleName, "driveOrientation=" + orientation);
            robot.robotDrive.driveBase.setDriveOrientation(
                orientation, orientation == TrcDriveBase.DriveOrientation.FIELD);
            if (robot.ledIndicator != null)
            {
                robot.ledIndicator.setDriveOrientation(orientation);
            }
        }
    }   //setDriveOrientation

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void driverButtonEvent(FtcGamepad.ButtonType button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "Driver: %s=%s", button, pressed? "Pressed": "Released");

        switch (button) {
            case A:
                // Toggle between field or robot oriented driving, only applicable for holonomic drive base.
//                if (driverAltFunc)
//                {
//                    if (pressed && robot.robotDrive != null)
//                    {
//                        if (robot.robotDrive.driveBase.isGyroAssistEnabled())
//                        {
//                            // Disable GyroAssist drive.
//                            robot.globalTracer.traceInfo(moduleName, ">>>>> Disabling GyroAssist.");
//                            robot.robotDrive.driveBase.setGyroAssistEnabled(null);
//                        }
//                        else
//                        {
//                            // Enable GyroAssist drive.
//                            robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling GyroAssist.");
//                            robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
//                        }
//                    }
//                }
//                else
//                {
//                if (pressed && robot.robotDrive != null && robot.robotDrive.driveBase.supportsHolonomicDrive()) {
//                    if (robot.robotDrive.driveBase.getDriveOrientation() != TrcDriveBase.DriveOrientation.FIELD) {
//                        robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling FIELD mode.");
//                        setDriveOrientation(TrcDriveBase.DriveOrientation.FIELD);
//                    } else {
//                        robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling ROBOT mode.");
//                        setDriveOrientation(TrcDriveBase.DriveOrientation.ROBOT);
//                    }
//                }
//                }
                break;

            case B:
//                robot.globalTracer.traceInfo(moduleName, ">>>>> DriverAltFunc=" + pressed);
//                driverAltFunc = pressed;
                if(pressed) {
                    robot.clawGrabber.open();
                    robot.globalTracer.traceInfo(moduleName, ">>>>> OpenClaw Closed=" + robot.clawGrabber.isClosed());
                }
                break;
            case X:
                if(pressed) {
                    robot.clawGrabber.close();
                    robot.globalTracer.traceInfo(moduleName, ">>>>> CloseClaw Closed=" + robot.clawGrabber.isClosed());
                }
                break;
            case Y:
                if(pressed) {
                    boolean isClose = robot.clawGrabber.isClosed();
                    robot.globalTracer.traceInfo(moduleName, ">>>> Y is pressed: closed=" + isClose);
                    if(isClose)
                    {
                        robot.clawGrabber.open();
                        robot.globalTracer.traceInfo(
                                moduleName, ">>>>> Opening claw: Closed=" + robot.clawGrabber.isClosed());
                    }
                    else
                    {
                        robot.clawGrabber.close();
                        robot.globalTracer.traceInfo(
                                moduleName, ">>>>> Closing claw: Closed=" + robot.clawGrabber.isClosed());
                    }
                }
//                if(pressed) {
//                    if(robot.clawGrabber.isClosed())
//                    {
//                        robot.clawGrabber.open();
//                        robot.globalTracer.traceInfo(moduleName, ">>>>> OpenClaw Closed=" + robot.clawGrabber.isClosed());
//                    }
//                    else
//                    {
//                        robot.clawGrabber.close();
//                        robot.globalTracer.traceInfo(moduleName, ">>>>> CloseClaw Closed=" + robot.clawGrabber.isClosed());
//                    }
//                }
                break;

            case LeftBumper:
                // Toggle claw open/close.
                if (pressed && robot.claw != null)
                {
                    if (robot.clawGrabber.isClosed())
                    {
                        robot.claw.getClawGrabber().open();
                    }
                    else {
                        if (isSamplePickupMode) {
                            isClawGrabbing = true;
                            robot.verticalWrist.setPosition(robot.vWristElevatorScaling()-0.08);
                            robot.arm.setPosition(robot.armElevatorScaling() -0.1);
                            robot.clawGrabber.close(null, .18, null);
                            runtime.reset();
                        }
                        else
                        {
                            robot.clawGrabber.close();
                        }
                    }
                }
                break;

            case RightBumper:
                break;

            case DpadUp:
                if(pressed)
                {
                    if(!isSampleTypeRedAlliance)
                    {
                        SamplePickupType = Claw.SamplePickupType.redAllianceSamples;
                        isSampleTypeRedAlliance = true;
                    }
                    else
                    {
                        SamplePickupType = Claw.SamplePickupType.blueAllianceSamples;
                        isSampleTypeRedAlliance = false;
                    }
                }
                break;
            case DpadDown:
                if (pressed && !robot.autoHang.isActive())
                {
                    isSamplePickupMode = false;
                    robot.autoHang.autoClimbLevel2(null);
                }
            case DpadLeft:
            case DpadRight:
                break;
            case Back:
                if (pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrate pressed.");
                    robot.cancelAll();
                    robot.zeroCalibrate(moduleName,null,null);
                    if (robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                    {
                        // Drive base is a Swerve Drive, align all steering wheels forward.
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Set SteerAngle to zero.");
                        ((FtcSwerveDrive) robot.robotDrive).setSteerAngle(0.0, false, false);
                    }
                }
                break;
            case Start:
                if (robot.vision != null && robot.vision.aprilTagVision != null && robot.robotDrive != null)
                {
                    // On press of the button, we will start looking for AprilTag for re-localization.
                    // On release of the button, we will set the robot's field location if we found the AprilTag.
                    relocalizing = pressed;
                    if (!pressed)
                    {
                        if (robotFieldPose != null)
                        {
                            // Vision found an AprilTag, set the new robot field location.
                            robot.globalTracer.traceInfo(
                                moduleName, ">>>>> Finish re-localizing: pose=" + robotFieldPose);
                            robot.robotDrive.driveBase.setFieldPosition(robotFieldPose, false);
                            robotFieldPose = null;
                        }
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Start re-localizing ...");
                    }
                }
                break;
        }
    }   //driverButtonEvent

    /**
     * This method is called when operator gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void operatorButtonEvent(FtcGamepad.ButtonType button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "Operator: %s=%s", button, pressed? "Pressed": "Released");

        switch (button)
        {
            case A:
            case B:
                break;
            case X:
                if(pressed && robot.verticalWrist != null)
                {
                    if(!is45Left) {
                        robot.verticalWrist.setPosition(RobotParams.WristParamsRotational.DEGREES_45_LEFT);
                        isWristRotatorMiddle = false;
                        is45Left = true;
                    }
                    else
                    {
                        robot.verticalWrist.setPosition(RobotParams.WristParamsRotational.DEGREES_45_RIGHT);
                        isWristRotatorMiddle = false;
                        is45Left = false;
                    }
                }
            case Y:
                break;
            case LeftBumper:
                if(pressed)
                {
                    if(!isSamplePickupMode)
                    {
                        //used
                        isSamplePickupMode = true;
                        robot.wristArm.setWristArmPosition(robot.armElevatorScaling(), robot.vWristElevatorScaling());
                    }
                    else
                    {
                        //used
                        isSamplePickupMode = false;
                        robot.wristArm.setWristArmBasketScorePos();
                        robot.rotationalWrist.setPosition(RobotParams.WristParamsRotational.MIDDLE_P0S);
                    }
                }
                break;
            case RightBumper:
                if(pressed && robot.verticalWrist != null)
                {
                    if(!isWristRotatorMiddle)
                    {
                        robot.verticalWrist.setPosition(RobotParams.WristParamsRotational.MIDDLE_P0S);
                        isWristRotatorMiddle = true;
                        is45Left = false;
                    }
                    else
                    {
                        robot.verticalWrist.setPosition(RobotParams.WristParamsRotational.MIN_P0S);
                        isWristRotatorMiddle = false;
                        is45Left = false;
                    }
                }
                break;
            case DpadUp:
                break;
            case DpadDown:
            case DpadLeft:
            case DpadRight:
                break;

            case Back:
                if (pressed)
                {
                    // Zero calibrate all subsystems (arm, elevator and turret).
                    robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrate pressed.");
                    robot.cancelAll();
                    robot.zeroCalibrate(moduleName,null,null);
                }
                break;

            case Start:
                break;
        }
    }   //operatorButtonEvent

}   //class FtcTeleOp
