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
     * This class contains Gobilda motor parameters.
     */
    public static class Gobilda
    {
        // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_312_ENC_PPR       = (((1.0 + 46.0/17.0)*(1.0 + 46.0/11.0))*28.0);
        public static final double MOTOR_5203_312_MAX_RPM       = 312.0;
        public static final double MOTOR_5203_312_MAX_VEL_PPS   =
            MOTOR_5203_312_ENC_PPR * MOTOR_5203_312_MAX_RPM / 60.0;     // 2795.9872 pps
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_435_ENC_PPR       = (((1.0 + 46.0/17.0)*(1.0 + 46.0/17.0))*28.0);
        public static final double MOTOR_5203_435_MAX_RPM       = 435.0;
        public static final double MOTOR_5203_435_MAX_VEL_PPS   =
            MOTOR_5203_435_ENC_PPR * MOTOR_5203_435_MAX_RPM / 60.0;     // 2787.9135 pps
    }   //class Gobilda

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
        public static final double APRILTAG_AUDIENCE_WALL_X         = -70.25;
        public static final double APRILTAG_BACK_WALL_X             = 70.25;
        public static final double APRILTAG_BLUE_ALLIANCE_WALL_Y    = 70.25;
        public static final double APRILTAG_RED_ALLIANCE_WALL_Y     = -70.25;
        public static final double APRILTAG_WALL_OFFSET_Y           = 46.83;
        public static final TrcPose2D[] APRILTAG_POSES              = new TrcPose2D[] {
                new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, APRILTAG_WALL_OFFSET_Y, -90.0), // TagId 11
                new TrcPose2D(0.0, APRILTAG_BLUE_ALLIANCE_WALL_Y, 0.0),                 // TagId 12
                new TrcPose2D(APRILTAG_BACK_WALL_X, APRILTAG_WALL_OFFSET_Y, 90.0),      // TagId 13
                new TrcPose2D(APRILTAG_BACK_WALL_X, -APRILTAG_WALL_OFFSET_Y, 90.0),     // TagId 14
                new TrcPose2D(0.0, APRILTAG_RED_ALLIANCE_WALL_Y, 180.0),                // TagId 15
                new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, -APRILTAG_WALL_OFFSET_Y, -90.0) // TagId 16
        };
    }   //class Game


    /**
     * This class contains miscellaneous robot info.
     */
    public static class Robot
    {
        public static final String TEAM_FOLDER_PATH             =
            Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftcTeam";
        public static final String LOG_FOLDER_PATH              = TEAM_FOLDER_PATH + "/tracelogs";
        public static final String STEER_ZERO_CAL_FILE          = TEAM_FOLDER_PATH + "/SteerZeroCalibration.txt";
        public static final double DASHBOARD_UPDATE_INTERVAL    = 0.1;      // in msec
        public static final String ROBOT_CODEBASE               = "Robot2025";
        public static final double ROBOT_LENGTH                 = 17.0;
        public static final double ROBOT_WIDTH                  = 17.0;
        // Robot Drive Parameters.
        public static final DriveMode DRIVE_MODE                = DriveMode.ArcadeMode;
        public static final DriveOrientation DRIVE_ORIENTATION  = DriveOrientation.ROBOT;
        public static final double DRIVE_SLOW_SCALE             = 0.3;
        public static final double DRIVE_NORMAL_SCALE           = 1.0;
        public static final double TURN_SLOW_SCALE              = 0.3;
        public static final double TURN_NORMAL_SCALE            = 0.6;
    }   //class Robot

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
        // Vision
        public static final boolean useVision                   = true;
        public static final boolean useWebCam                   = true; // false to use Android phone camera.
        public static final boolean useBuiltinCamBack           = false; // For Android Phone as Robot Controller.
        public static final boolean tuneColorBlobVision         = false;
        public static final boolean useLimelightVision          = true;
        public static final boolean useCameraStreamProcessor    = true;
        public static final boolean useAprilTagVision           = true;
        public static final boolean useColorBlobVision          = true;
        public static final boolean showVisionView              = !inCompetition;
        public static final boolean showVisionStat              = false;
        // Drive Base
        public static final boolean useDriveBase                = false;
        public static final boolean usePinpointOdometry         = false;
        public static final boolean useSparkfunOTOS             = false;
        // Subsystems
        public static final boolean useSubsystems               = false;
        public static final boolean useElevator                 = false;
        public static final boolean useClaw                     = false;
        public static final boolean useArm                      = false;
        public static final boolean useTurret                   = false;
        public static final boolean useWristVertical            = false;
        public static final boolean useWristRotational           = false;
        public static final boolean useElbow                    = false;
    }   //class Preferences

    //
    // Robot Parameters.
    //

    /**
     * This class contains the parameters of the front camera.
     */
    public static class FrontCamParams extends FtcRobotDrive.VisionInfo
    {
        public FrontCamParams()
        {
            camName = "WebCam 1";
            camImageWidth = 1920;
            camImageHeight = 1080;
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
     * This class contains the Differential Robot Parameters.
     */
    public static class DifferentialParams extends FtcRobotDrive.RobotInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public DifferentialParams()
        {
            robotName = "DifferentialRobot";
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
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor"};
            driveMotorInverted = new boolean[] {true, false};
            odometryType = TrcDriveBase.OdometryType.MotorOdometry;
            // Odometry Wheels
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = null;
            xOdWheelIndices = new int[] {FtcRobotDrive.INDEX_RIGHT_BACK};
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = null;
            yOdWheelIndices = new int[] {FtcRobotDrive.INDEX_LEFT_FRONT, FtcRobotDrive.INDEX_RIGHT_FRONT};
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            absoluteOdometry = null;
            // Drive Motor Odometry
            yDrivePosScale = 0.02166184604662450653409090909091;        // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 23.0;        // inches/sec
            robotMaxAcceleration  = 500.0;  // inches/sec2
            robotMaxTurnRate = 100.0;       // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            yDrivePidCoeffs = new PidCoefficients(0.06, 0.0, 0.002, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = new FrontCamParams();
            webCam2 = new BackCamParams();
            limelight = new LimelightParams();
            // Miscellaneous
            blinkinName = "blinkin";
        }   //DifferentialParams
    }   //class DifferentialParams

    /**
     * This class contains the Mecanum Robot Parameters.
     */
    public static class MecanumParams extends FtcRobotDrive.RobotInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
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
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            odometryType = TrcDriveBase.OdometryType.OdometryWheels;
            // Odometry Wheels
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = new String[] {"xOdWheelSensor"};
            xOdWheelIndices = new int[] {0};
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = new String[] {"yLeftOdWheelSensor", "yRightOdWheelSensor"};
            yOdWheelIndices = new int[] {1, 2};
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            if (odometryType == TrcDriveBase.OdometryType.AbsoluteOdometry)
            {
                if (RobotParams.Preferences.usePinpointOdometry)
                {
                    FtcPinpointOdometry.Config ppOdoConfig = new FtcPinpointOdometry.Config()
                            .setPodOffsets(0.0, 0.0)    // ???
                            .setEncoderResolution(ODWHEEL_CPR / Math.PI * ODWHEEL_DIAMETER)
                            .setEncodersInverted(false, false); //???
                    absoluteOdometry = new FtcPinpointOdometry("pinpointOdo", ppOdoConfig);
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
            robotMaxVelocity = 23.0;        // inches/sec
            robotMaxAcceleration  = 500.0;  // inches/sec2
            robotMaxTurnRate = 100.0;       // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = new PidCoefficients(0.95, 0.0, 0.001, 0.0, 0.0);
            xDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = null;
            yDrivePidCoeffs = new PidCoefficients(0.06, 0.0, 0.002, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = new FrontCamParams();
            webCam2 = null; //new BackCamParams()
            limelight = null; //new LimelightParams()
            // Miscellaneous
            blinkinName = null;
        }   //MecanumParams
    }   //class MecanumParams

    /**
     * This class contains the Swerve Drive Base Parameters.
     */
    public static class SwerveParams extends FtcSwerveDrive.SwerveInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public SwerveParams()
        {
            robotName = "SwerveRobot";
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
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            odometryType = TrcDriveBase.OdometryType.OdometryWheels;
            // Odometry Wheels
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = null;
            xOdWheelIndices = new int[] {FtcRobotDrive.INDEX_RIGHT_BACK};
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = null;
            yOdWheelIndices = new int[] {FtcRobotDrive.INDEX_LEFT_FRONT, FtcRobotDrive.INDEX_RIGHT_FRONT};
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            absoluteOdometry = null;
            // Drive Motor Odometry
            xDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            yDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 23.0;        // inches/sec
            robotMaxAcceleration  = 500.0;  // inches/sec2
            robotMaxTurnRate = 100.0;       // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = yDrivePidCoeffs = new PidCoefficients(0.95, 0.0, 0.001, 0.0, 0.0);
            xDrivePidPowerLimit = yDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = new FrontCamParams();
            webCam2 = new BackCamParams();
            limelight = new LimelightParams();
            // Miscellaneous
            blinkinName = "blinkin";
            // Steer Encoders
            steerEncoderNames = new String[] {"lfSteerEncoder", "rfSteerEncoder", "lbSteerEncoder", "rbSteerEncoder"};
            steerEncoderInverted = new boolean[] {false, false, false, false};
            steerEncoderZeros = new double[] {0.474812, 0.467663, 0.541338, 0.545340};
            steerZerosFilePath = Robot.STEER_ZERO_CAL_FILE;
            // Steer Motors
            steerMotorType = MotorType.CRServo;
            steerMotorNames = new String[] {"lfSteerServo", "rfSteerServo", "lbSteerServo", "rbSteerServo"};
            steerMotorInverted = new boolean[] {true, true, true, true};
            steerMotorPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 0.0, 0.0);
            steerMotorPidTolerance = 1.0;
            // Swerve Modules
            swerveModuleNames = new String[] {"lfWheel", "rfWheel", "lbWheel", "rbWheel"};
        }   //SwerveParams
    }   //class SwerveParams

    //
    // Subsystems.
    //
    public static final class ElevatorParams
    {
        public static final String SUBSYSTEM_NAME                          = "Elevator";

        public static final String PRIMARY_MOTOR_NAME                       = SUBSYSTEM_NAME + ".Primary";
        public static final String FOLLOWER_MOTOR_NAME                      = SUBSYSTEM_NAME + ".Follower";
        public static final MotorType PRIMARY_MOTOR_TYPE                    = MotorType.DcMotor;
        public static final MotorType FOLLOWER_MOTOR_TYPE                   = MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED                  = false;
        public static final boolean FOLLOWER_MOTOR_INVERTED                 = false;

        public static final double INCHES_PER_COUNT                         = 1; //Need to be Updated
        public static final double POS_OFFSET                               = 0; //Need to be Updated
        public static final double POWER_LIMIT                              = 1.0; //Need to be Updated
        public static final double ZERO_CAL_POWER                           = -0.25; //Need to be Updated

        public static final double MIN_POS                                  = POS_OFFSET; //Need to be Updated
        public static final double MAX_POS                                  = 48; //Need to be Updated
        public static final double[] POS_PRESETS                            = {MIN_POS, MAX_POS}; //Need to be Updated
        public static final double POS_PRESET_TOLERANCE                     = 1.0;

        public static final boolean SOFTWARE_PID_ENABLED                        = true;
        public static final TrcPidController.PidCoefficients POS_PID_COEFFS =
                new TrcPidController.PidCoefficients(1.0,0,0,0,0); //Need to tune
        public static final double POS_PID_TOLERANCE                        = 0.2;
        public static final double GRAVITY_COMP_POWER                       = 0.0; //Need to be Updated
        public static final double STALL_MIN_POWER                          = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE                          = 0.1;
        public static final double STALL_TIMEOUT                            = 0.1;
        public static final double STALL_RESET_TIMEOUT                      = 0.0;
    } //Elevator

    public static final class ClawParams
    {
        public static final String SUBSYSTEM_NAME               = "Grabber";

        public static final String PRIMARY_SERVO_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final double OPEN_POS                     = 0; //Need to update
        public static final double OPEN_TIME                    = 0.2;
        public static final double CLOSE_POS                    = 0; //Need to update
        public static final double CLOSE_TIME                   = 0.2;

        public static final boolean USE_REV_V3_COLOR_SENSOR     = true;
        public static final String REV_V3_COLOR_SENSOR_NAME     = SUBSYSTEM_NAME + ".sensor";
        public static final double SENSOR_TRIGGER_THRESHOLD     = 2.0; //cm
        public static final double HAS_OBJECT_THRESHOLD         = 2.0; //cm
        public static final boolean ANALOG_TRIGGER_INVERTED     = false;

    }   //class Grabber

    public static final class ArmParams
    {
        public static final String SUBSYSTEM_NAME                           = "Arm";
        public static final String PRIMARY_SERVO_NAME                       = SUBSYSTEM_NAME + ".Primary";
        public static final String FOLLOWER_SERVO_NAME                      = SUBSYSTEM_NAME + ".Follower";
        public static final MotorType PRIMARY_SERVO_TYPE                    = MotorType.CRServo;
        public static final MotorType FOLLOWER_SERVO_TYPE                   = MotorType.CRServo;
        public static final boolean PRIMARY_SERVO_INVERTED                  = false;
        public static final boolean FOLLOWER_SERVO_INVERTED                 = false;

        public static final double DEGREES_PER_COUNT                        = 1; //Need to be Updated
        public static final double POS_OFFSET                               = 0; //Need to be Updated
        public static final double POWER_LIMIT                              = 1.0; //Need to be Updated

        public static final String EXTERNAL_ENCODER_NAME                    = SUBSYSTEM_NAME + ".encoder";
        public static final boolean EXTERNAL_ENCODER_INVERTED               = false;

        public static final double MIN_POS                                  = POS_OFFSET; //Need to be Updated
        public static final double MAX_POS                                  = 10; //Need to be Updated
        public static final double[] POS_PRESETS                            = {MIN_POS, MAX_POS}; //Need to be Updated
        public static final double POS_PRESET_TOLERANCE                     = 0.0;

        public static final double SAMPLE_PICKUP_POS                        = 0; //Need to be Updated
        public static final double SAMPLE_DROP_POS                          = 0; //Need to be Updated
        public static final double SPECIMEN_PICKUP                          = 0; //Need to be Updated
        public static final double SPECIMEN_DROP                            = 0; //Need to be Updated
        public static final double PICKUP_TIME                              = 0.5; //Need to be Updated
        public static final double DROP_TIME                                = 0.5; //Need to be Updated
    }

    public static class ElbowParams
    {
        public static final String SUBSYSTEM_NAME               = "Elbow";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final MotorType PRIMARY_MOTOR_TYPE        = MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;

        public static final double GOBILDA312_CPR               = Gobilda.MOTOR_5203_312_ENC_PPR; //Need to be Updated
        public static final double DEG_SCALE                    = 360.0 / GOBILDA312_CPR; //Need to be Updated
        public static final double POS_OFFSET                   = 0; //Need to be Updated
        public static final double ZERO_OFFSET                  = 0.0; //Need to be Updated
        public static final double POWER_LIMIT                  = 1.0; //Need to be Updated
        public static final double ZERO_CAL_POWER               = -0.25; //Need to be Updated

        public static final double MIN_POS                      = POS_OFFSET; //Need to be Updated
        public static final double MAX_POS                      = 130; //Need to be Updated
        public static final double[] posPresets                 = {MIN_POS, 90.0,100,110, 120.0, MAX_POS}; //Need to be Updated
        public static final double POS_PRESET_TOLERANCE         = 10.0; //Need to be Updated

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients POS_PID_COEFFS =
                new TrcPidController.PidCoefficients(0.018, 0.1, 0.001, 0.0, 2.0); //Need to tune
        public static final double POS_PID_TOLERANCE            = 0.3;
        public static final double GRAVITY_COMP_MAX_POWER       = 0; //Need to be Updated
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class ElbowParams

    public static final class TurretParams
    {
        public static final String SUBSYSTEM_NAME                           = "Turret";
        public static final String PRIMARY_SERVO_NAME                       = SUBSYSTEM_NAME + ".Primary";
        public static final boolean PRIMARY_SERVO_INVERTED                  = false;

        public static final double DEGREES_PER_COUNT                        = 1; //Need to be Updated
        public static final double POS_OFFSET                               = 90; //Need to be Updated
        public static final double POWER_LIMIT                              = 1.0; //Need to be Updated

        public static final String EXTERNAL_ENCODER_NAME                    = SUBSYSTEM_NAME + ".encoder";
        public static final boolean EXTERNAL_ENCODER_INVERTED               = false;

        public static final double MIN_POS                                  = 0; //Need to be Updated
        public static final double MAX_POS                                  = 180; //Need to be Updated

    }   //class Turret

    public static class WristParamsVertical
    {
        public static final String SUBSYSTEM_NAME                             = "WristVertical";
        public static final String PRIMARY_SERVO_VERTICAL                   = SUBSYSTEM_NAME + ".primary";

        public static final boolean PRIMARY_SERVO_VERTICAL_INVERTED         = false;

        public static final double LOGICAL_MIN_POS                          = 0.0;
        public static final double LOGICAL_MAX_POS                          = 1.0;
        public static final double PHYSICAL_MIN_POS                         = 0.0;
        public static final double PHYSICAL_MAX_POS                         = 1.0;
        public static final double MAX_STEPRATE                             = 1.0;

        public static final double POS_OFFSET                               = 0;
        public static final double MIN_POS                                  = POS_OFFSET;
        public static final double MAX_POS                                  = 50;
    }   //class WristParamsVertical

    public static class WristParamsRotational
    {
        public static final String SUBSYSTEM_NAME                             = "WristRotator";
        public static final String PRIMARY_SERVO_ROTATOR                   = SUBSYSTEM_NAME + ".primary";

        public static final boolean PRIMARY_SERVO_ROTATOR_INVERTED         = false;

        public static final double LOGICAL_MIN_POS                          = 0.0;
        public static final double LOGICAL_MAX_POS                          = 1.0;
        public static final double PHYSICAL_MIN_POS                         = 0.0;
        public static final double PHYSICAL_MAX_POS                         = 1.0;
        public static final double MAX_STEPRATE                             = 1.0;

        public static final double POS_OFFSET                               = 0;
        public static final double MIN_POS                                  = POS_OFFSET;
        public static final double MAX_POS                                  = 50;
    }   //class WristParamsRotational

}   //class RobotParams
