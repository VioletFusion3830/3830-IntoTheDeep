package teamcode.subsystems;

import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

public class Arm {
    private final TrcServo arm;


    public Arm(){
        FtcServoActuator.Params armParams = new FtcServoActuator.Params()
                .setPrimaryServo(RobotParams.ArmParams.PRIMARY_SERVO_NAME,
                        RobotParams.ArmParams.PRIMARY_SERVO_INVERTED)
                .setFollowerServo(RobotParams.ArmParams.FOLLOWER_SERVO_NAME,
                        RobotParams.ArmParams.FOLLOWER_SERVO_INVERTED)
                .setLogicalPosRange(RobotParams.ArmParams.LOGICAL_MIN_POS,
                        RobotParams.ArmParams.LOGICAL_MAX_POS)
                .setPhysicalPosRange(RobotParams.ArmParams.PHYSICAL_MIN_POS,
                        RobotParams.ArmParams.PHYSICAL_MAX_POS)
                .setMaxStepRate(RobotParams.ArmParams.MAX_STEPRATE)
                .setPositionPresets(RobotParams.ArmParams.POS_PRESET_TOLERANCE,
                        RobotParams.ArmParams.POS_PRESETS);
        arm = new FtcServoActuator(armParams).getServo();
    }

    public TrcServo getArm()
    {
        return arm;
    }

}
