package teamcode.subsystems;

import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Objects;

import teamcode.RobotParams;
import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcServoGrabber;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcServoGrabber;

public class Claw {
    private final TrcServoGrabber claw; //one servo for open/close
    private final RevColorSensorV3 revColorSensorV3;

    private final ColorRange yellowSampleHue = new ColorRange(60,90);
    private final ColorRange blueSampleHue = new ColorRange(180,240);
    private final ColorRange redSampleHue = new ColorRange(330,30);
    private SamplePickupType samplePickupType = SamplePickupType.yellowSample;

    public enum SamplePickupType
    {
        redAllianceSamples,
        blueAllianceSamples,
        redSample,
        blueSample,
        yellowSample,
        anySample
    }

    public enum SampleSensorColor
    {
        redSample,
        blueSample,
        yellowSample,
        noSampleColor
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
                    this::getSensorDataDistance,
                    RobotParams.ClawParams.ANALOG_TRIGGER_INVERTED,
                    RobotParams.ClawParams.SENSOR_TRIGGER_THRESHOLD,
                    RobotParams.ClawParams.HAS_OBJECT_THRESHOLD,
                    isSampleCorrectColor(getSensorDataColor(),samplePickupType),
                    true
                    );
        }

        claw = new FtcServoGrabber(RobotParams.ClawParams.SUBSYSTEM_NAME, grabberParams).getGrabber();
        claw.open();
    }

    public TrcServoGrabber getClaw()
    {
        return claw;
    }

    public SamplePickupType getSamplePickupType()
    {
        return samplePickupType;
    }

    public SamplePickupType setSamplePickupType(SamplePickupType newSamplePickupType)
    {
        return samplePickupType = newSamplePickupType;
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

    private SampleSensorColor getSensorDataColor()
    {
        if (revColorSensorV3 != null)
        {
            float[] hsvValues = {0F,0F,0F};
            Color.RGBToHSV(revColorSensorV3.red() * 255,
                    revColorSensorV3.green() * 255,
                    revColorSensorV3.blue() * 255,
                    hsvValues);
            float hue = hsvValues[0];

            if(yellowSampleHue.isHueInRange(hue))
            {
                return SampleSensorColor.yellowSample;
            }
            else if(redSampleHue.isHueInRange(hue))
            {
                return SampleSensorColor.redSample;
            }
            else if(blueSampleHue.isHueInRange(hue))
            {
                return SampleSensorColor.blueSample;
            }
            else
            {
                return SampleSensorColor.noSampleColor;
            }
        }
        else
        {
            return null;
        }
    }

    public TrcEvent.Callback isSampleCorrectColor(SampleSensorColor sampleSensorColor, SamplePickupType samplePickupType)
    {
        if(claw != null) {
            if (revColorSensorV3 != null) {
                switch (sampleSensorColor) {
                    case redSample:
                        if (samplePickupType == SamplePickupType.redAllianceSamples ||
                                samplePickupType == SamplePickupType.redSample ||
                                samplePickupType == SamplePickupType.anySample) {
                            claw.close();
                        }
                        break;
                    case blueSample:
                        if (samplePickupType == SamplePickupType.blueAllianceSamples ||
                                samplePickupType == SamplePickupType.blueSample ||
                                samplePickupType == SamplePickupType.anySample) {
                            claw.close();
                        }
                        break;
                    case yellowSample:
                        if (samplePickupType == SamplePickupType.redAllianceSamples ||
                                samplePickupType == SamplePickupType.blueAllianceSamples ||
                                samplePickupType == SamplePickupType.yellowSample ||
                                samplePickupType == SamplePickupType.anySample) {
                            claw.close();
                        }
                        break;
                    case noSampleColor:
                        break;
                }
            }
            else
            {
                claw.close();
            }
        }
        return null;
    }

    // Helper class to manage color ranges.
    private static class ColorRange {
        private final int minHue;
        private final int maxHue;

        public ColorRange(int minHue, int maxHue) {
            this.minHue = minHue;
            this.maxHue = maxHue;
        }

        public boolean isHueInRange(float hue) {
            if (minHue <= maxHue) {
                return hue >= minHue && hue <= maxHue;
            } else {
                return hue >= minHue || hue <= maxHue;
            }
        }
    }
}   //class Grabber
