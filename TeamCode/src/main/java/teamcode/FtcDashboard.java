package teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import trclib.robotcore.TrcPidController;

@Config
public class FtcDashboard
{
    @Config
    public static class ServoTune
    {
        public static double ServoA = 0.5;
        public static double ServoB = 0.5;
        public static double ServoC = 0.5;
        public static double ServoD = 0.5;
    }

    @Config
    public static class TunePID
    {
        public static TrcPidController.PidCoefficients tunePidCoeff = new TrcPidController.PidCoefficients(0, 0, 0.0, 0,0);
        public static double GarvityComp = 0.0;;
    }

    @Config
    public static class PPTuneParams
    {
        public static double tuneDistance = 4;
        public static double tuneAngleDistance = 90;
        public static double powerLimit = 1;
        public static double maxVel = 65;
        public static double maxAccel = 90;
        public static double kfMaxVel = 90;
    }
    public static TrcPidController.PidCoefficients pPPidCoeff = new TrcPidController.PidCoefficients(0, 0, 0.0, 0,0);
}
