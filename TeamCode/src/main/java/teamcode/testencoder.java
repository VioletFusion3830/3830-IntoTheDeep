package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic", group="Linear OpMode")
public class testencoder extends LinearOpMode {

    CRServo servo;
    DcMotor motor;

    @Override
    public void runOpMode() {

        AnalogInput analogInput = hardwareMap.get(AnalogInput.class,"arm.encoder");
        servo = hardwareMap.get(CRServo.class, "arm.primary");
        motor = hardwareMap.get(DcMotor.class,"elevator.primary");

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double position = analogInput.getVoltage() / analogInput.getMaxVoltage();
            if(gamepad1.a)
            {
                servo.setPower(.5);
            }
            else
            {
                servo.setPower(0);
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("position", position);
            telemetry.update();
        }
    }}