package teamcode.subsystems;

import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcServoGrabber;
import teamcode.RobotParams;
import trclib.subsystem.TrcServoGrabber;

public class Claw {
    private final TrcServoGrabber claw; //one servo for open/close
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

        if (revColorSensorV3 != null)
        {
            grabberParams.setAnalogSensorTrigger(
                    this::getSensorDataDistance, RobotParams.ClawParams.ANALOG_TRIGGER_INVERTED,
                    RobotParams.ClawParams.SENSOR_TRIGGER_THRESHOLD, RobotParams.ClawParams.HAS_OBJECT_THRESHOLD,
                    null);
        }

        claw = new FtcServoGrabber(RobotParams.ClawParams.SUBSYSTEM_NAME, grabberParams).getGrabber();
        claw.open();
    }

    public TrcServoGrabber getClaw()
    {
        return claw;
    }

    private double getSensorDataDistance()
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

    private double getSensorDataColor() {
        if (revColorSensorV3 != null)
        {
            return revColorSensorV3.getLightDetected();
        } else
        {
            return 0.0;
        }
    }
}   //class Grabber
