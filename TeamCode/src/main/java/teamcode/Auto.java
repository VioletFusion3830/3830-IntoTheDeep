package teamcode;
import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Auto")
public class Auto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();
    double integralSum = 0;
    private double lastError = 0;
    double outputLimit = 1;
    DcMotor motor;
    CRServo servo;
    Servo servoreal;
    DcMotor leftFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightFrontDrive;
    DcMotor rightBackDrive;
    private double Kp = .8;
    private double Ki = 0;
    private double Kd = 0;

    @Override
    public void runOpMode()
    {
        motor = hardwareMap.get(DcMotor.class, "elbow.primary");
        AnalogInput analogInput = hardwareMap.get(AnalogInput.class,"arm.encoder");
        servo = hardwareMap.get(CRServo.class, "arm.primary");
        servoreal = hardwareMap.get(Servo.class,"wristVertical.primary");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lfDriveMotor");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lbDriveMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rfDriveMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rbDriveMotor");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeInInit())
        {
            motor.setTargetPosition(650);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(.4);
            telemetry.addData("pos", motor.getCurrentPosition());
            telemetry.update();
            servoreal.setPosition(RobotParams.WristParamsVertical.SAMPLE_PICKUP_POS);
        }
        runtime2.reset();
        while(opModeIsActive())
        {
            double position = analogInput.getVoltage() / analogInput.getMaxVoltage();
            servo.setPower(-PIDControl(.660,position));
            telemetry.addData("time", runtime2.seconds());
            telemetry.update();
            servoreal.setPosition(RobotParams.WristParamsVertical.SPECIMEN_PICKUP_POS);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(.1);
            if(runtime2.seconds() > 1)
            {
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
            }
            else
            {
                leftFrontDrive.setPower(-0.2);
                rightFrontDrive.setPower(-0.2);
                leftBackDrive.setPower(-0.2);
                rightBackDrive.setPower(-0.2);
            }
        }

    }

        public double PIDControl(double refrence, double state) {
            double error = (refrence - state);
            integralSum += error * runtime.seconds();
            double derivative = (error - lastError) / (runtime.seconds());
            lastError = error;
            runtime.reset();
            double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
            return Range.clip(output,-outputLimit,outputLimit);
        }
}

