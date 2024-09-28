package teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcServoGrabber;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

public class Claw {
    private final TrcServo claw; //one servo for open/close
    private final RevColorSensorV3 revColorSensorV3;

    public Claw()
    {
        if (RobotParams.ClawParams.USE_REV_V3_COLOR_SENSOR)
        {
            revColorSensorV3 = FtcOpMode.getInstance().hardwareMap.get(
                    RevColorSensorV3.class, RobotParams.ClawParams.REV_V3_COLOR_SENSOR_NAME);
        }
        else
        {
            revColorSensorV3 = null;
        }

        FtcServoGrabber.Params grabberParams = new FtcServoGrabber.Params()
                .setPrimaryServo(RobotParams.ClawParams.PRIMARY_SERVO_NAME, RobotParams.ClawParams.PRIMARY_SERVO_INVERTED)
                .setOpenCloseParams(RobotParams.ClawParams.OPEN_POS, RobotParams.ClawParams.OPEN_TIME,
                        RobotParams.ClawParams.CLOSE_POS, RobotParams.ClawParams.CLOSE_TIME);

        if (rev2mSensor != null)
        {
            grabberParams.setAnalogSensorTrigger(
                    this::getSensorData, RobotParams.ClawParams.ANALOG_TRIGGER_INVERTED,
                    RobotParams.ClawParams.SENSOR_TRIGGER_THRESHOLD, RobotParams.ClawParams.HAS_OBJECT_THRESHOLD,
                    null);
        }
        else if (RobotParams.ClawParams.USE_DIGITAL_SENSOR)
        {
            grabberParams.setDigitalInputTrigger(
                    RobotParams.ClawParams.DIGITAL_SENSOR_NAME, RobotParams.ClawParams.DIGITAL_TRIGGER_INVERTED, null);
        }

        grabber = new FtcServoGrabber(RobotParams.ClawParams.SUBSYSTEM_NAME, grabberParams).getGrabber();
        grabber.open();
    }

    public TrcServoGrabber getGrabber()
    {
        return grabber;
    }

    private double getSensorData()
    {
        if (revColorSensorV3 != null)
        {
            return revColorSensorV3.getDistance(DistanceUnit.CM);
        }
        else
        {
            return 0.0;
        }
    }

}   //class Grabber
