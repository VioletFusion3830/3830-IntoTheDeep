package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import ftclib.robotcore.FtcOpMode;
import trclib.robotcore.TrcRobot;

@Autonomous(name="Auto")
public class Auto extends FtcOpMode {

    Robot robot;

    @Override
    public void robotInit() {
        robot = new Robot(TrcRobot.getRunMode());
    }

    @Override
    public void runOpMode()
    {
        robot.elbow.setPosition(RobotParams.ElbowParams.PICKUP_SAMPLE_POS);
        robot.arm.setPosition(RobotParams.ArmParams.PICKUP_SAMPLE_POS);
        robot.wristVertical.setPosition(RobotParams.WristParamsVertical.SAMPLE_PICKUP_POS);
        robot.wristRotational.setPosition(RobotParams.WristParamsRotational.MIN_P0S);
        robot.clawServo.open();
    }
}

