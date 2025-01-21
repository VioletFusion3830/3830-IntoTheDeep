package teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

import trclib.robotcore.TrcRobot;

@TeleOp(name = "DashboardTest")
public class FtcDashboardTest extends LinearOpMode {

    int i;

    public void runOpMode()
    {
        //robot = new Robot(null);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive() && !isStopRequested())
        {
//            while(gamepad1.a)
//            {
//                i++;
//            }
//
//            while(!gamepad1.a)
//            {
//                i--;
//            }
            i++;
            telemetry.addData("I Value:", i);
            telemetry.update();
        }
    }
}



