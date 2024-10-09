package teamcode.subsystems;

import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

public class WristVertical {

    private final TrcServo wristVertical;

    public WristVertical()
    {
        FtcServoActuator.Params WristVerticalParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                        RobotParams.WristParamsVertical.SUBSYSTEM_NAME,
                        RobotParams.WristParamsVertical.PRIMARY_SERVO_VERTICAL_INVERTED)
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
        wristVertical = new FtcServoActuator(WristVerticalParams).getServo();
    }

    public TrcServo getWristVertical()
    {
        return wristVertical;
    }

}
