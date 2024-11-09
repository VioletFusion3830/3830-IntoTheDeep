package teamcode;

import com.acmerobotics.dashboard.config.Config;
import trclib.robotcore.TrcPidController;

public class FtcDashboard
{
    @Config
    public static class TunePID
    {
        public static TrcPidController.PidCoefficients tunePidCoeff = new TrcPidController.PidCoefficients(0.4, 0.1, 0.004, 0,5);
    }
    @Config
    public static class TunePID_Secondary
    {
        public static TrcPidController.PidCoefficients tunePidCoeff = new TrcPidController.PidCoefficients(0.045, 0.2, 0.005, 0.0, 5.0);
    }

    @Config
    public static class PPTuneParams
    {
        public static double tuneDistance = 8;
        public static double tuneAngleDistance = 90;
        public static double powerLimit = 0.7;
    }
}
