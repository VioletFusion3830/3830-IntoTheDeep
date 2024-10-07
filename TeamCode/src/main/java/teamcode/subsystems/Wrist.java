package teamcode.subsystems;

import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

public class Wrist {

    private final TrcServo wristVertical;
    private final TrcServo wristRotator;

    public Wrist {
        FtcServoActuator.Params WristVerticalParams = new FtcServoActuator.Params()
                .setPrimaryServo(RobotParams.WristParamsVertical.SUBSYSTEM_NAME,
                        RobotParams.WristParamsVertical.PRIMARY_SERVO_VERTICAL_INVERTED)
                .setLogicalPosRange(RobotParams.WristParamsVertical.LOGICAL_MIN_POS,
                        RobotParams.WristParamsVertical.LOGICAL_MAX_POS)
                .setPhysicalPosRange(RobotParams.WristParamsVertical.PHYSICAL_MIN_POS,
                        RobotParams.WristParamsVertical.PHYSICAL_MAX_POS)
                .setMaxStepRate(RobotParams.WristParamsVertical.MAX_STEPRATE)
                .setPositionPresets(RobotParams.WristParamsVertical.POS_PRESET_TOLERANCE,
                        RobotParams.WristParamsVertical.POS_PRESETS);
        wristVertical = new FtcServoActuator(WristVerticalParams).getServo();

        FtcServoActuator.Params WristRotatorParams = new FtcServoActuator.Params()
                .setPrimaryServo(RobotParams.WristParamsVertical.SUBSYSTEM_NAME,
                        RobotParams.WristParamsVertical.PRIMARY_SERVO_VERTICAL_INVERTED)
                .setLogicalPosRange(RobotParams.WristParamsVertical.LOGICAL_MIN_POS,
                        RobotParams.WristParamsVertical.LOGICAL_MAX_POS)
                .setPhysicalPosRange(RobotParams.WristParamsVertical.PHYSICAL_MIN_POS,
                        RobotParams.WristParamsVertical.PHYSICAL_MAX_POS)
                .setMaxStepRate(RobotParams.WristParamsVertical.MAX_STEPRATE)
                .setPositionPresets(RobotParams.WristParamsVertical.POS_PRESET_TOLERANCE,
                        RobotParams.WristParamsVertical.POS_PRESETS);
        wristRotator = new FtcServoActuator(WristRotatorParams).getServo();
    }

    public TrcServo getWristVertical() {return wristVertical;}
    public TrcServo getWristRotator() {return wristRotator;}

}
