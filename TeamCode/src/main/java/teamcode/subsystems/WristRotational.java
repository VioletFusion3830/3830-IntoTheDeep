package teamcode.subsystems;

import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

public class WristRotational {
    private final TrcServo wristRotatorServo;

    public WristRotational()
    {
        FtcServoActuator.Params WristRotatorParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR,
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR_INVERTED);
        wristRotatorServo = new FtcServoActuator(WristRotatorParams).getServo();
    }

    public TrcServo getWristRotationalServo()
    {
        return wristRotatorServo;
    }
}
