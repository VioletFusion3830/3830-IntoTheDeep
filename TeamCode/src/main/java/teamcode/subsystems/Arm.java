package teamcode.subsystems;

import ftclib.motor.FtcMotorActuator;
import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;

public class Arm {
    private final TrcMotor arm;

    public Arm(){
        FtcMotorActuator.Params armParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(RobotParams.ArmParams.PRIMARY_SERVO_NAME,
                        RobotParams.ArmParams.PRIMARY_SERVO_TYPE,
                        RobotParams.ArmParams.PRIMARY_SERVO_INVERTED)
                .setFollowerMotor(RobotParams.ArmParams.FOLLOWER_SERVO_NAME,
                        RobotParams.ArmParams.FOLLOWER_SERVO_TYPE,
                        RobotParams.ArmParams.FOLLOWER_SERVO_INVERTED)
                .setPositionScaleAndOffset(RobotParams.ArmParams.DEGREES_PER_COUNT,
                        RobotParams.ElevatorParams.POS_OFFSET)
                .setExternalEncoder(RobotParams.ArmParams.EXTERNAL_ENCODER_NAME,
                        RobotParams.ArmParams.EXTERNAL_ENCODER_INVERTED)
                .setPositionPresets(RobotParams.ArmParams.POS_PRESET_TOLERANCE,
                        RobotParams.ArmParams.POS_PRESETS);
        arm = new FtcMotorActuator(armParams).getMotor();
    }

    public TrcMotor getArm()
    {
        return arm;
    } //get CRServo

    //Need to add external encoder code here.

}