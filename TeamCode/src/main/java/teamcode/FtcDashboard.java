package teamcode;

import com.acmerobotics.dashboard.config.Config;

import trclib.controller.TrcPidController;

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
        public static TrcPidController.PidCoefficients tunePidCoeff = new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0,0);
        public static double GarvityComp = 0.0;;
    }

    @Config
    public static class PPTuneParams
    {
        public static double tuneDistance = 3;
        public static double tuneAngleDistance = 90;
        public static double powerLimit = 1;
        public static double maxVel = 70;
        public static double maxAccel = 100;
        public static double maxDecel = 80;
        public static double kfMaxVel = 90;
    }
    public static TrcPidController.PidCoefficients pPPidCoeff = new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0,0);
}
