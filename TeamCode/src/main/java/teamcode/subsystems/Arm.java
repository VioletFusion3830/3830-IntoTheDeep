package teamcode.subsystems;

import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

public class Arm {
//    three servos:
//    two for vertical rotation
//    one for horizontal rotation
    private final TrcServo arm; //Pitch rotation
    private final TrcServo armRotator; //Yaw rotation


    public Arm(){
        FtcServoActuator.Params armParams = new FtcServoActuator.Params()
                .setPrimaryServo(RobotParams.ArmParams.PRIMARY_SERVO_NAME, RobotParams.ArmParams.PRIMARY_SERVO_INVERTED)
                .setFollowerServo(RobotParams.ArmParams.FOLLOWER_SERVO_NAME, RobotParams.ArmParams.FOLLOWER_SERVO_INVERTED)
                .setLogicalPosRange(RobotParams.ArmParams.MIN_POS, RobotParams.ArmParams.MAX_POS)
                .setPhysicalPosRange(RobotParams.ArmParams.MIN_POS, RobotParams.ArmParams.MAX_POS)
                .setMaxStepRate(RobotParams.ArmParams.MAX_STEPRATE)
                .setPositionPresets(RobotParams.ArmParams.POS_PRESET_TOLERANCE, RobotParams.ArmParams.POS_PRESETS);

        FtcServoActuator.Params armRotatorParams = new FtcServoActuator.Params()
                .setPrimaryServo(RobotParams.ArmParams.PRIMARY_SERVO_NAME, RobotParams.ArmParams.PRIMARY_SERVO_INVERTED)
                .setLogicalPosRange(RobotParams.ArmParams.ROTATE_MIN_POS, RobotParams.ArmParams.ROTATE_MAX_POS)
                .setPhysicalPosRange(RobotParams.ArmParams.ROTATE_MIN_POS, RobotParams.ArmParams.ROTATE_MAX_POS)
                .setMaxStepRate(RobotParams.ArmParams.MAX_STEPRATE)
                .setPositionPresets(RobotParams.ArmParams.POS_PRESET_TOLERANCE, RobotParams.ArmParams.ROTATION_POS_PRESETS);

        arm = new FtcServoActuator(armParams).getServo();
        armRotator = new FtcServoActuator(armRotatorParams).getServo();
    }

    public TrcServo getArm() {return arm;}
    public TrcServo getArmRotator() {return armRotator;}
}
