/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import android.os.Environment;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.*;

import org.openftc.easyopencv.OpenCvCameraRotation;

import ftclib.drivebase.FtcRobotDrive;
import ftclib.drivebase.FtcSwerveDrive;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.sensor.FtcSparkFunOtos;
import ftclib.sensor.FtcPinpointOdometry;
import ftclib.sensor.FtcUltrasonicSensor;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.driverio.TrcGameController.DriveMode;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcPidController;
import trclib.robotcore.TrcPidController.PidCoefficients;
import trclib.vision.TrcHomographyMapper;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams
{
    /**
     * When the season starts, the competition robot may not be ready for programmers. It's crucial to save time by
     * developing code on robots of previous seasons. By adding previous robots to the list of RobotType, one can
     * easily switch the code to handle different robots.
     */
    public enum RobotType
    {
        // This is useful for developing Vision code where all you need is a Control Hub and camera.
        VisionOnly,
        // Generic Differential Drive Base Robot
        DifferentialRobot,
        // Generic Mecanum Drive Base Robot
        MecanumRobot,
        // Generic Swerve Drive Base Robot
        SwerveRobot
    }   //enum RobotType

    public static class VisionOnlyParams extends FtcRobotDrive.RobotInfo
    {
        public VisionOnlyParams()
        {
            robotName = "VisionOnly";
            // Front Camera
            webCam1 = new FrontCamParams();
            // Back Camera
            webCam2 = new BackCamParams();
            limelight = new LimelightParams();
        }   //VisionOnlyParams
    }   //class VisionOnlyParams

    /**
     * This class contains robot preferences. It enables/disables various robot features. This is especially useful
     * during robot development where some subsystems may not be available or ready yet. By disabling unavailable
     * subsystems, one can test the rest of the robot without the fear of code crashing when some subsystems are not
     * found.
     */
    public static class Preferences
    {
        // Global config
        public static final RobotType robotType                 = RobotType.MecanumRobot;
        public static final boolean inCompetition               = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useLoopPerformanceMonitor   = true;
        public static final boolean useBatteryMonitor           = false;
        // Status Update: Status Update may affect robot loop time, don't do it when in competition.
        public static final boolean doStatusUpdate              = !inCompetition;
        public static final boolean showSubsystems              = true;
        public static final boolean useBlinkinLED               = false;
        public static final boolean useGobildaLED               = false;
        // Vision
        public static final boolean useVision                   = false;
        public static final boolean useWebCam                   = false; // false to use Android phone camera.
        public static final boolean useBuiltinCamBack           = false; // For Android Phone as Robot Controller.
        public static final boolean tuneColorBlobVision         = false;
        public static final boolean useLimelightVision          = false;
        public static final boolean useCameraStreamProcessor    = false;
        public static final boolean useAprilTagVision           = false;
        public static final boolean useColorBlobVision          = false;
        public static final boolean showVisionView              = !inCompetition;
        public static final boolean showVisionStat              = false;
        // Drive Base
        public static final boolean useDriveBase                = true;
        public static final boolean usePinpointOdometry         = true;
        public static final boolean useSparkfunOTOS             = false;
        // Subsystems
        public static final boolean useSubsystems               = true;
        public static final boolean useElevator                 = true;
        public static final boolean useElbow                    = true;
        public static final boolean useClaw                     = true;
        public static final boolean useWristRotational          = true;
        public static final boolean useWristArm                 = true;
    }   //class Preferences

    /**
     * This class contains field dimension constants. Generally, these should not be changed.
     */
    public static class Field
    {
        public static final double FULL_FIELD_INCHES            = 141.24;
        public static final double HALF_FIELD_INCHES            = FULL_FIELD_INCHES/2.0;
        public static final double FULL_TILE_INCHES             = FULL_FIELD_INCHES/6.0;
    }   //class Field

    /**
     * This class contains season specific game element information.
     */
    public static class Game
    {
        // DO NOT CHANGE the AprilTag location numbers. They are from the AprilTag metadata.
        // All AprilTags are at the height of 5.75-inch from the tile floor.
        public static final double APRILTAG_AUDIENCE_WALL_X                 = -70.25;
        public static final double APRILTAG_BACK_WALL_X                     = 70.25;
        public static final double APRILTAG_BLUE_ALLIANCE_WALL_Y            = 70.25;
        public static final double APRILTAG_RED_ALLIANCE_WALL_Y             = -70.25;
        public static final double APRILTAG_WALL_OFFSET_Y                   = 46.83;
        public static final TrcPose2D[] APRILTAG_POSES                      = new TrcPose2D[] {
                new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, APRILTAG_WALL_OFFSET_Y, -90.0), // TagId 11
                new TrcPose2D(0.0, APRILTAG_BLUE_ALLIANCE_WALL_Y, 0.0),                 // TagId 12
                new TrcPose2D(APRILTAG_BACK_WALL_X, APRILTAG_WALL_OFFSET_Y, 90.0),      // TagId 13
                new TrcPose2D(APRILTAG_BACK_WALL_X, -APRILTAG_WALL_OFFSET_Y, 90.0),     // TagId 14
                new TrcPose2D(0.0, APRILTAG_RED_ALLIANCE_WALL_Y, 180.0),                // TagId 15
                new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, -APRILTAG_WALL_OFFSET_Y, -90.0) // TagId 16
        };

        public static final double AUTO_PERIOD                      = 30.0; // 30 seconds auto period
        public static final double TELEOP_PERIOD                    = 120.0;// 2 minutes teleop period
        public static final double SCORE_BASKET_CYCLE_TIME          = 6.0;  // in seconds
        public static final double CHAMBER_LENGTH                   = 26.0;
        public static final double CHAMBER_MAX_SCORE_POS_X          = (CHAMBER_LENGTH / 2.0);

        // Blue alliance positions will be derived using adjustPoseByAlliance.
        // Robot start locations in inches.
        public static final double STARTPOS_X                               = Robot.ROBOT_WIDTH/2.0;
        public static final double STARTPOS_Y                               = Field.HALF_FIELD_INCHES - Robot.ROBOT_LENGTH/2.0;
        // Red Net Zone start pose face the net zone touching the alliance wall with the robot's in front of net zone.
        public static final TrcPose2D STARTPOSE_RED_NET_ZONE                = new TrcPose2D(-(Field.HALF_FIELD_INCHES-(Field.FULL_TILE_INCHES+Robot.ROBOT_LENGTH/2.0)), -Robot.ROBOT_WIDTH/2.0, -90);
        // Red Observation Zone start pose face forwards robot 1 in form center tile and touch back wall.
        public static final TrcPose2D STARTPOSE_RED_OBSERVATION_ZONE        = new TrcPose2D(Robot.ROBOT_WIDTH/2.0+1, -(Field.HALF_FIELD_INCHES - Robot.ROBOT_LENGTH/2.0), 180);

        // Score poses (Net zone side).
        public static final TrcPose2D RED_BASKET_SCORE_POSE         =
                new TrcPose2D(0, 0, 0.0);
        public static final TrcPose2D RED_NET_CHAMBER_SCORE_POSE    =
                new TrcPose2D(-7, Field.FULL_TILE_INCHES, 180);
        // Score pose (Observation zone side).
        public static final TrcPose2D RED_OBSERVATION_CHAMBER_SCORE_POSE =
                new TrcPose2D(7, Field.FULL_TILE_INCHES, 180);
        // Pickup pose (Net zone side).
        public static final TrcPose2D RED_NET_ZONE_SPIKEMARK_PICKUP =
                new TrcPose2D(0,0, 0.0);
        // Pickup pose (Observation zone side).
        public static final TrcPose2D RED_OBSERVATION_ZONE_PICKUP   =
                new TrcPose2D(Field.HALF_FIELD_INCHES-(Field.FULL_TILE_INCHES+Field.FULL_TILE_INCHES/2),Field.HALF_FIELD_INCHES/2, 180);
        // Park pose (Net zone side).
        public static final TrcPose2D RED_ASCENT_ZONE_PARK_POSE     =
                new TrcPose2D(0,0, 0.0);
        // Park pose (Observation zone side).
        public static final TrcPose2D RED_OBSERVATION_ZONE_PARK_POSE=
                new TrcPose2D(0, 0, 0.0);


    }   //class Game

    //
    // Robot Parameters.
    //

    /**
            * This class contains miscellaneous robot info.
     */
    public static class Robot
    {
        public static final String TEAM_FOLDER_PATH                         =
                Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftcTeam";
        public static final String LOG_FOLDER_PATH                          = TEAM_FOLDER_PATH + "/tracelogs";
        public static final String STEER_ZERO_CAL_FILE                      = TEAM_FOLDER_PATH + "/SteerZeroCalibration.txt";
        public static final double DASHBOARD_UPDATE_INTERVAL                = 0.1; // in msec
        public static final String ROBOT_CODEBASE                           = "Robot2025";
        public static final double ROBOT_LENGTH                             = 15.125;
        public static final double ROBOT_WIDTH                              = 16.25;
        // Robot Drive Parameters.
        public static final DriveMode DRIVE_MODE                = DriveMode.ArcadeMode;
        public static final DriveOrientation DRIVE_ORIENTATION  = DriveOrientation.FIELD;
        public static final double DRIVE_SLOW_SCALE                         = 0.5;
        public static final double DRIVE_NORMAL_SCALE                       = 1.0;
        public static final double TURN_SLOW_SCALE                          = 0.5;
        public static final double TURN_NORMAL_SCALE                        = 0.8;
    }   //class Robot

    /**
     * This class contains the parameters of the front camera.
     */
    public static class FrontCamParams extends FtcRobotDrive.VisionInfo
    {
        public FrontCamParams()
        {
            camName = "WebCam 1";
            camImageWidth = 640;
            camImageHeight = 480;
            camXOffset = 0.0;                   // Inches to the right from robot center
            camYOffset = 0;                   // Inches forward from robot center
            camZOffset = 0;                  // Inches up from the floor
            camPitch = 0;                    // degrees down from horizontal
            camYaw = 0.0;                       // degrees clockwise from robot front
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                0.0, 120.0,                                             // Camera Top Left
                camImageWidth -1, 120.0,                                // Camera Top Right
                0.0, camImageHeight - 1,                                // Camera Bottom Left
                camImageWidth - 1, camImageHeight - 1);                 // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -12.5626, 48.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Left
                11.4375, 44.75 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Right
                -2.5625, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,    // World Bottom Left
                2.5626, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset);    // World Bottom Right
        }   //FrontCamParams
    }   //class FrontCamParams

    /**
     * This class contains the parameters of the back camera.
     */
    public static class BackCamParams extends FtcRobotDrive.VisionInfo
    {
        public BackCamParams()
        {
            camName = null;
            camImageWidth = 640;
            camImageHeight = 480;
            camXOffset = 0.0;                   // Inches to the right from robot center
            camYOffset = 0;                   // Inches forward from robot center
            camZOffset = 0;                  // Inches up from the floor
            camPitch = 0;                    // degrees down from horizontal
            camYaw = 0.0;                       // degrees clockwise from robot front
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                0.0, 120.0,                                             // Camera Top Left
                camImageWidth -1, 120.0,                                // Camera Top Right
                0.0, camImageHeight - 1,                                // Camera Bottom Left
                camImageWidth - 1, camImageHeight - 1);                 // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -12.5626, 48.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Left
                11.4375, 44.75 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Right
                -2.5625, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,    // World Bottom Left
                2.5626, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset);    // World Bottom Right
        }   //BackCamParams
    }   //class BackCamParams

    /**
     * This class contains the parameters of the Limelight vision processor.
     */
    public static class LimelightParams extends FtcRobotDrive.VisionInfo
    {
        public LimelightParams()
        {
            camName = null;
            camImageWidth = 640;
            camImageHeight = 480;
            camXOffset = 0.0;                   // Inches to the right from robot center
            camYOffset = 0;                   // Inches forward from robot center
            camZOffset = 0;                  // Inches up from the floor
            camPitch = 0;                    // degrees down from horizontal
            camYaw = 0.0;                       // degrees clockwise from robot front
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                0.0, 0.0,                                             // Camera Top Left
                camImageWidth - 1, 0.0,                                // Camera Top Right
                0.0, camImageHeight - 1,                                // Camera Bottom Left
                camImageWidth - 1, camImageHeight - 1);                 // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -12.5626, 48.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Left
                11.4375, 44.75 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Right
                -2.5625, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,    // World Bottom Left
                2.5626, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset);    // World Bottom Right
        }   //LimelightParams
    }   //class LimelightParams

    /**
     * This class contains the Mecanum Robot Parameters.
     */
    public static class MecanumParams extends FtcRobotDrive.RobotInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER_MM = 35.0;
        private static final double ODWHEEL_DIAMETER = ODWHEEL_DIAMETER_MM*TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public MecanumParams()
        {
            robotName = "MecanumRobot";
            // Robot Dimensions
            robotLength = Robot.ROBOT_LENGTH;
            robotWidth = Robot.ROBOT_WIDTH;
            wheelBaseLength = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = 16.0;
            // IMU
            imuName = "imu";
            hubLogoDirection = LogoFacingDirection.UP;
            hubUsbDirection = UsbFacingDirection.FORWARD;
            // Drive Motors
            driveMotorType = MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor","lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            odometryType = TrcDriveBase.OdometryType.AbsoluteOdometry;
            // Odometry Wheels
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = null;
            xOdWheelIndices = new int[] {1};
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = null;
            yOdWheelIndices = new int[] {0, 3};
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            if (odometryType == TrcDriveBase.OdometryType.AbsoluteOdometry)
            {
                if (RobotParams.Preferences.usePinpointOdometry)
                {
                    FtcPinpointOdometry.Config ppOdoConfig = new FtcPinpointOdometry.Config()
                            .setPodOffsets(180, -24) //180,-24
                            .setEncoderResolution(ODWHEEL_CPR / (Math.PI * ODWHEEL_DIAMETER_MM))
                            .setEncodersInverted(false, true); //???
                    absoluteOdometry = new FtcPinpointOdometry("pinpointOdo", ppOdoConfig);
                    headingWrapRangeLow = -180.0;
                    headingWrapRangeHigh = 180.0;
                }
                else if (RobotParams.Preferences.useSparkfunOTOS)
                {
                    FtcSparkFunOtos.Config otosConfig = new FtcSparkFunOtos.Config()
                            .setOffset(0.0, 0.0, 0.0)   //???
                            .setScale(1.0, 1.0);        //???
                    absoluteOdometry = new FtcSparkFunOtos("sparkfunOtos", otosConfig);
                }
            }
            else
            {
                absoluteOdometry = null;
            }
            // Drive Motor Odometry
            xDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            yDrivePosScale = 0.02166184604662450653409090909091;        // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 60;        // inches/sec
            robotMaxAcceleration  = 250;  // inches/sec2
            robotMaxTurnRate = 100;       // degrees/sec
            profiledMaxVelocity = 60;
            profiledMaxAcceleration = 250;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = new PidCoefficients(0.17, 0.0, 0.015, 0.0, 0.0);
            xDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = null;
            yDrivePidCoeffs = new PidCoefficients(0.08, 0.0, 0.008, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.042, 0.0, 0.0018, 0.0, 0.0);
            turnPidPowerLimit = 0.6;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = null; //new FrontCamParams()
            webCam2 = null; //new BackCamParams()
            limelight = null; //new LimelightParams()
            // Miscellaneous
            indicatorName = null;
        }   //MecanumParams
    }   //class MecanumParams

    //
    // Subsystems.
    //
    public static final class ElevatorParams
    {
        public static final String SUBSYSTEM_NAME                           = "elevator";

        public static final String PRIMARY_MOTOR_NAME                       = SUBSYSTEM_NAME + ".primary";
        public static final String FOLLOWER_MOTOR_NAME                      = SUBSYSTEM_NAME + ".follower";
        public static final MotorType PRIMARY_MOTOR_TYPE                    = MotorType.DcMotor;
        public static final MotorType FOLLOWER_MOTOR_TYPE                   = MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED                  = true;
        public static final boolean FOLLOWER_MOTOR_INVERTED                 = true;

        public static final double INCHES_PER_COUNT                         = 0.016481481481;
        public static final double POS_OFFSET                               = 12;
        public static final double POWER_LIMIT                              = 1.0;
        public static final double ZERO_CAL_POWER                           = -0.25;
        public static final double HORIZONTAL_LIMIT                         = 28;

        public static final double MIN_POS                                  = POS_OFFSET;
        public static final double MIN_POS_ELBOW_UP                         = 14.65;
        public static final double MAX_POS                                  = 58;
        public static final double PICKUP_SAMPLE_POS                        = 12.5;
        public static final double PICKUP_SPECIMEN_POS                      = 26.5;
        public static final double LOW_BASKET_SCORE_POS                     = 32;
        public static final double HIGH_BASKET_SCORE_POS                    = 45;
        public static final double HIGH_CHAMBER_SCORE_POS                   = 22.5;
        public static final double LEVEL1_ASCENT_POS                        = 45;
        public static final double LEVEL2_ASCENT_START_POS                  = 31;
        public static final double LEVEL2_ASCENT_POS                        = MIN_POS_ELBOW_UP;
        public static final double[] POS_PRESETS                            = {13,30,45};
        public static final double POS_PRESET_TOLERANCE                     = 5.0;

        public static final boolean SOFTWARE_PID_ENABLED                        = true;
        public static final TrcPidController.PidCoefficients PID_COEFFS     =
                new TrcPidController.PidCoefficients(0.28,0.4, 0.012, 0,3); //Need to tune
        public static final double POS_PID_TOLERANCE                        = 0.5;

        public static final double MAX_GRAVITY_COMP_POWER                   = 0.08;
        public static final double STALL_MIN_POWER                          = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE                          = 0.1;
        public static final double STALL_TIMEOUT                            = 0.1;
        public static final double STALL_RESET_TIMEOUT                      = 0.0;
    } //Elevator

    public static class ElbowParams
    {
        public static final String SUBSYSTEM_NAME                           = "elbow";

        public static final String PRIMARY_MOTOR_NAME                       = SUBSYSTEM_NAME + ".primary";
        public static final MotorType PRIMARY_MOTOR_TYPE                    = MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED                  = false;

        public static final double DEG_PER_COUNT                            = .0555;
        public static final double POS_OFFSET                               = 7;
        public static final double POWER_LIMIT                              = 1.0;
        public static final double ZERO_CAL_POWER                           = -0.25;
        public static final double RESTRICTED_POS_THRESHOLD                 = 50; //Angle in degrees

        public static final double MIN_POS                                  = 10;
        public static final double MAX_POS                                  = 100;
        public static final double PICKUP_SAMPLE_POS                        = 10;
        public static final double PICKUP_SPECIMEN_POS                      = 10;
        public static final double BASKET_SCORE_POS                         = 92;
        public static final double HIGH_CHAMBER_SCORE_POS                   = 97;
        public static final double LEVEL1_ASCENT_POS                        = 30;
        public static final double LEVEL2_ASCENT_START_POS                  = 115;
        public static final double LEVEL2_ASCENT_POS                        = 80;
        public static final double[] POS_PRESETS                            = {10,30,60,90}; //Need to be Updated
        public static final double POS_PRESET_TOLERANCE                     = 5.0;

        public static final boolean SOFTWARE_PID_ENABLED                    = true;
        public static final TrcPidController.PidCoefficients PID_COEFFS     =
                new TrcPidController.PidCoefficients(0.07, 0.1, 0.004, 0.0, 4); //Need to tune
        public static final double PID_TOLERANCE                            = 1.0;

        public static final double MAX_GRAVITY_COMP_AT_MIN_SLIDER_LENGTH    = 0.04;
        public static final double STALL_MIN_POWER                          = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE                          = 0.1;
        public static final double STALL_TIMEOUT                            = 0.1;
        public static final double STALL_RESET_TIMEOUT                      = 0.0;
    }   //class ElbowParams

    public static final class ArmParams
    {
        public static final String SUBSYSTEM_NAME                           = "arm";
        public static final String PRIMARY_SERVO_NAME                       = SUBSYSTEM_NAME + ".primary";
        public static final MotorType PRIMARY_SERVO_TYPE                    = MotorType.CRServo;
        public static final boolean PRIMARY_SERVO_INVERTED                  = true;

        public static final double LOGICAL_MIN_POS                          = 0.83;
        public static final double LOGICAL_MAX_POS                          = 0.00;
        public static final double PHYSICAL_MIN_POS                         = 0;
        public static final double PHYSICAL_MAX_POS                         = 0;
        public static final double MAX_STEP_RATE                            = 0; //TBD

        public static final double PICKUP_SPECIMEN_POS                      = 0.5;
        public static final double BASKET_SCORE_POS                         = 0.600; //TBD
        public static final double HIGH_CHAMBER_SCORE_POS                   = 0.6;
        public static final double PICKUP_SAMPLE_POS_BASE                   = 0.6; //need to be found for low
        public static final double SAMPLE_PICKUP_MODE_START                 = 0.460;
        public static final double SAMPLE_PICKUP_MODE_SCALE                 = 0.060; //0.418
    }

    public static class WristParamsVertical
    {
        public static final String SUBSYSTEM_NAME                             = "wristVertical";
        public static final String PRIMARY_SERVO_VERTICAL                   = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_VERTICAL_INVERTED         = false;

        public static final double PICKUP_SAMPLE_POS_BASE                        = 0.760; //558
        public static final double PICKUP_SPECIMEN_POS                      = 0.5;
        public static final double BASKET_SCORE_POS                         = 0.25;
        public static final double HIGH_CHAMBER_SCORE_POS                   = 0.24;
        public static final double SAMPLE_PICKUP_MODE_START                 = 0.771;
        public static final double SAMPLE_PICKUP_MODE_SCALE                 = 0.030; //.745
    }   //class WristParamsVertical

    public static final class ClawParams
    {
        public static final String SUBSYSTEM_NAME                           = "grabber";

        public static final String PRIMARY_SERVO_NAME                       = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_INVERTED                  = false;

        public static final double OPEN_POS                                 = 0.775; //486
        public static final double OPEN_TIME                                = 0.2;
        public static final double CLOSE_POS                                = 1;
        public static final double CLOSE_TIME                               = 0.2;

        public static final boolean USE_REV_V3_COLOR_SENSOR                 = true;
        public static final String REV_V3_COLOR_SENSOR_NAME                 = SUBSYSTEM_NAME + ".sensor";
        public static final double SENSOR_TRIGGER_THRESHOLD                 = 2.3; //cm
        public static final boolean ANALOG_TRIGGER_INVERTED                 = true;

    }   //class Grabber

    public static class WristParamsRotational
    {
        public static final String SUBSYSTEM_NAME                           = "wristRotator";
        public static final String PRIMARY_SERVO_ROTATOR                    = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_ROTATOR_INVERTED          = false;
        //Need to be program servo and re get positions
        public static final double MIDDLE_P0S                               = 0.57;
        public static final double MIDDLE_POS2                              = 0.005;
        public static final double DEGREES_45                               = 0.125;
        public static final double MIN_P0S                                  = 0.308;
        public static final double[] POS_PRESETS                            = {MIDDLE_P0S, MIN_P0S};
    }   //class WristParamsRotational

}   //class RobotParams