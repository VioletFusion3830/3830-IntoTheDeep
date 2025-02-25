package teamcode;

import com.acmerobotics.dashboard.config.Config;

import trclib.controller.TrcPidController;
import trclib.vision.TrcOpenCvColorBlobPipeline;

@Config
public class FtcDashboard
{
    @Config
    public static class ServoTune
    {
        public static double ServoA = 0.55;
        public static double ServoB = 0.5;
        public static double ServoC = 0.5;
        public static double ServoD = 0.5;
    }

    @Config
    public static class TuneVision
    {
        public static double[] tuneSampleColorThreshold = {100.0, 250.0, 120.0, 200.0, 30.0, 80.0};
        public static double minArea = 1000;
        public static double maxArea = 100000;
        public static double minPerimeter = 100;
        public static double minWidthRange = 20;
        public static double maxWidthRange = 180;
        public static double minHeightRange = 20;
        public static double maxHeightRange = 180;
        public static double aspectRatioA = 0.4;
        public static double aspectRatioB = 2.5;
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
        public static double maxAccel = 160;
        public static double maxDecel = 110;
        public static double kfMaxVel = 90;
    }
    public static TrcPidController.PidCoefficients pPPidCoeff = new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0,0);
}
