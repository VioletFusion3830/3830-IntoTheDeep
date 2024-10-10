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
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR,
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR_INVERTED)
                .setLogicalPosRange(
                        RobotParams.WristParamsRotational.LOGICAL_MIN_POS,
                        RobotParams.WristParamsRotational.LOGICAL_MAX_POS)
                .setPhysicalPosRange(
                        RobotParams.WristParamsRotational.PHYSICAL_MIN_POS,
                        RobotParams.WristParamsRotational.PHYSICAL_MAX_POS)
                .setMaxStepRate(
                        RobotParams.WristParamsRotational.MAX_STEPRATE);
        wristRotator = new FtcServoActuator(WristRotatorParams).getServo();
    }

    public TrcServo getWristRotational()
    {
        return wristRotator;
    }
}
