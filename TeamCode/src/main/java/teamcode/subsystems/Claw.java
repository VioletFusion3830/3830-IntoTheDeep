package teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import java.util.Objects;

import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcServoGrabber;
import teamcode.RobotParams;
import trclib.subsystem.TrcServoGrabber;

public class Claw {
    private final TrcServoGrabber claw; //one servo for open/close
    private final RevColorSensorV3 revColorSensorV3;
    private double redValue;
    private double blueValue;
    private double yellowValue;
    private double alphaValue; //Light Intensity
    private String samplePickupType = "yellow";

//    private enum SamplePickupType
//    {
//        RedSample,
//        BlueSample,
//        YellowSample,
//        RedAllianceSamples,
//        BlueAllianceSamples,
//        AnySample
//    }

    public enum SampleSensorColor
    {
        RedSample,
        BlueSample,
        YellowSample
    }

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
                    RobotParams.ClawParams.SENSOR_TRIGGER_THRESHOLD, RobotParams.ClawParams.HAS_OBJECT_THRESHOLD,null
                    );
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
    public boolean isCorrectColor(SampleSensorColor sampleSensorColor, String samplePickupType)
    {
        if(revColorSensorV3 != null)
        {
            switch (sampleSensorColor)
            {
                case RedSample:
                    if(Objects.equals(samplePickupType, "redAllianceSamples") || Objects.equals(samplePickupType, "redSample") || Objects.equals(samplePickupType, "anySample"))
                    {
                        return true;
                    } else {
                        return false;
                    }
                    break;
                case BlueSample:
                    if(Objects.equals(samplePickupType, "blueAllianceSamples") || Objects.equals(samplePickupType, "blueSample") || Objects.equals(samplePickupType, "anySample"))
                    {
                        return true;
                    } else {
                        return false;
                    }
                    break;
                case YellowSample:
                    if(Objects.equals(samplePickupType, "redAllianceSamples") || Objects.equals(samplePickupType, "blueAllianceSamples") || Objects.equals(samplePickupType, "anySample"))
                    {
                        return true;
                    } else {
                        return false;
                    }
                    break;
            }
        }
    }

    private NormalizedRGBA getSensorDataColor() {
        if (revColorSensorV3 != null)
        {
            return revColorSensorV3.getNormalizedColors();
        } else
        {
            return null;
        }
    }

    public void getColor() {
        redValue = revColorSensorV3.red();
        blueValue = revColorSensorV3.blue();
        //yellowValue =revColorSensorV3.
        alphaValue = revColorSensorV3.alpha();
        Color.RGBToHSV();
    }
}   //class Grabber
