package teamcode;

import com.acmerobotics.dashboard.config.Config;
import trclib.robotcore.TrcPidController;

public class FtcDashboard
{
    @Config
    public static class TunePID
    {
        public static TrcPidController.PidCoefficients tunePidCoeff = new TrcPidController.PidCoefficients(.10, 0.0, 0.01, 0,0);
    }

    @Config
    public static class PPTuneParams
    {
        public static double tuneDistance = 8;
        public static double tuneAngleDistance = 90;
        public static double powerLimit = 0.7;
    }
}