package teamcode.subsystems;

import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import teamcode.Robot;
import teamcode.RobotParams;
import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcServoGrabber;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcServoGrabber;

public class Claw {
    private final TrcServoGrabber clawServo; //one servo for open/close
    private final RevColorSensorV3 revColorSensorV3;
    private final Robot robot;

    private final ColorRange yellowSampleHue = new ColorRange(60,90);
    private final ColorRange blueSampleHue = new ColorRange(200,240);
    private final ColorRange redSampleHue = new ColorRange(5,30);

    public enum SamplePickupType
    {
        redAllianceSamples,
        blueAllianceSamples,
        redSample,
        blueSample,
        yellowSample,
        anySample
    }

    public Claw(Robot robot)
    {
        this.robot = robot;
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
                .setPrimaryServo(RobotParams.ClawParams.PRIMARY_SERVO_NAME,
                        RobotParams.ClawParams.PRIMARY_SERVO_INVERTED)
                .setOpenCloseParams(RobotParams.ClawParams.OPEN_POS,
                        RobotParams.ClawParams.OPEN_TIME,
                        RobotParams.ClawParams.CLOSE_POS,
                        RobotParams.ClawParams.CLOSE_TIME);
        if (revColorSensorV3 != null)
        {
            grabberParams.setAnalogSensorTrigger(
                    this::getSensorDataDistance,
                    RobotParams.ClawParams.ANALOG_TRIGGER_INVERTED,
                    RobotParams.ClawParams.SENSOR_TRIGGER_THRESHOLD);
        }
        clawServo = new FtcServoGrabber(RobotParams.ClawParams.SUBSYSTEM_NAME, grabberParams).getGrabber();
        clawServo.tracer.setTraceLevel(TrcDbgTrace.MsgLevel.DEBUG);
        clawServo.open();
    }

    public TrcServoGrabber getClawServo()
    {
        return clawServo;
    }

    public void autoAssistPickup(String owner, double delay, TrcEvent event, double timeout, SamplePickupType sampleType)
    {
        clawServo.autoGrab(owner, delay, event, timeout, this::isSampleColorCorrect, sampleType);
    }

    public double getSensorDataDistance()
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

    //Will only work if rev color sensor is never null when called
    public double getSensorDataColor()
    {
        float[] hsvValues = {0.0f,0.0f,0.0f};
        NormalizedRGBA normalizedColors = revColorSensorV3.getNormalizedColors();
            Color.RGBToHSV((int) (normalizedColors.red * 255),
                    (int) (normalizedColors.green * 255),
                    (int) (normalizedColors.blue * 255),
                    hsvValues);
            return hsvValues[0];
    }

    private void isSampleColorCorrect(Object context)
    {
        if(!clawServo.isClosed() && clawServo.sensorTriggered()) {
            SamplePickupType sampleType = (SamplePickupType) context;
            double sampleHue = getSensorDataColor();
            boolean sampleColorCorrect = false;

            switch (sampleType) {
                case redSample:
                    sampleColorCorrect = redSampleHue.isHueInRange(sampleHue);
                    break;
                case blueSample:
                    sampleColorCorrect = blueSampleHue.isHueInRange(sampleHue);
                    break;
                case yellowSample:
                    sampleColorCorrect = yellowSampleHue.isHueInRange(sampleHue);
                    break;
                case redAllianceSamples:
                    sampleColorCorrect = yellowSampleHue.isHueInRange(sampleHue) || redSampleHue.isHueInRange(sampleHue);
                    robot.globalTracer.traceInfo(null, "Red Alliance Samples: " + sampleColorCorrect + ", isClawClosed: " + clawServo.isClosed() + ", sampleHue: " + sampleHue + ", isAutoActive: " + clawServo.isAutoActive());
                    break;
                case blueAllianceSamples:
                    sampleColorCorrect = yellowSampleHue.isHueInRange(sampleHue) || blueSampleHue.isHueInRange(sampleHue);
                    robot.globalTracer.traceInfo(null, "Blue Alliance Samples: " + sampleColorCorrect + ", isClawClosed: " + clawServo.isClosed() + ", sampleHue: " + sampleHue + ", isAutoActive: " + clawServo.isAutoActive());
                    break;
                case anySample:
                    sampleColorCorrect = yellowSampleHue.isHueInRange(sampleHue) || blueSampleHue.isHueInRange(sampleHue) || redSampleHue.isHueInRange(sampleHue);
                    break;
            }
            if (sampleColorCorrect) {
                clawServo.close();
                clawServo.cancel();
            }
        }
    }

    // Helper class to manage color ranges.
    private static class ColorRange
    {
        private final int minHue;
        private final int maxHue;

        public ColorRange(int minHue, int maxHue)
        {
            this.minHue = minHue;
            this.maxHue = maxHue;
        }

        public boolean isHueInRange(double hue)
        {
            if (minHue <= maxHue)
            {
                return hue >= minHue && hue <= maxHue;
            }
            else
            {
                return hue >= minHue || hue <= maxHue;
            }
        }
    }
}   //class Grabber