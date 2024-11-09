package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import trclib.robotcore.TrcRobot;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
public class testAxonEncoder extends LinearOpMode {

    protected Robot robot;

    @Override
    public void runOpMode() {


        robot = new Robot(TrcRobot.getRunMode());

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.a)
            {
                robot.wristArm.setWristArmSamplePickup();
                robot.elevator.setPosition(RobotParams.ElevatorParams.PICKUP_SAMPLE_POS);
                robot.elbow.setPosition(RobotParams.ElbowParams.PICKUP_SAMPLE_POS);
            }
            if(gamepad1.b)
            {
                robot.wristArm.setWristArmSpecimenPickup();
                robot.elevator.setPosition(RobotParams.ElevatorParams.PICKUP_SPECIMEN_POS);
                robot.elbow.setPosition(RobotParams.ElbowParams.PICKUP_SPECIMEN_POS);
            }
            if(gamepad1.x)
            {
                robot.wristArm.setWristArmSampleDrop();
                robot.elevator.setPosition(RobotParams.ElevatorParams.DROP_SAMPLE_POS);
                robot.elbow.setPosition(RobotParams.ElbowParams.DROP_SAMPLE_POS);
            }
            if(gamepad1.y)
            {
                robot.wristArm.setWristArmSpecimenDrop();
                robot.elevator.setPosition(RobotParams.ElevatorParams.DROP_SPECIMEN_POS);
                robot.elbow.setPosition(RobotParams.ElbowParams.DROP_SPECIMEN_POS);
            }

        }
    }}
