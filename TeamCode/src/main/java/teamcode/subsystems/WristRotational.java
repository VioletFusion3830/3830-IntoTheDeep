package teamcode.subsystems;

import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

public class WristRotational {
    private final TrcServo wristRotator;

    public WristRotational()
    {
        FtcServoActuator.Params WristRotatorParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                        RobotParams.WristParamsVertical.SUBSYSTEM_NAME,
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR_INVERTED)
                .setLogicalPosRange(
                        RobotParams.WristParamsVertical.LOGICAL_MIN_POS,
                        RobotParams.WristParamsVertical.LOGICAL_MAX_POS)
                .setPhysicalPosRange(
                        RobotParams.WristParamsVertical.PHYSICAL_MIN_POS,
                        RobotParams.WristParamsVertical.PHYSICAL_MAX_POS)
                .setMaxStepRate(
                        RobotParams.WristParamsVertical.MAX_STEPRATE)
                .setPositionPresets(
                        RobotParams.WristParamsVertical.POS_PRESET_TOLERANCE,
                        RobotParams.WristParamsVertical.POS_PRESETS);
        wristRotator = new FtcServoActuator(WristRotatorParams).getServo();
    }

    public TrcServo getWristRotational()
    {
        return wristRotator;
    }
}
