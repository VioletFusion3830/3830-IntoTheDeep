package teamcode.subsystems;

import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

public class RotationalWrist {
    private final TrcServo rotationalWristServo;

    public RotationalWrist()
    {
        FtcServoActuator.Params rotationalWristParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR,
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR_INVERTED);
        rotationalWristServo = new FtcServoActuator(rotationalWristParams).getServo();
    }

    public TrcServo getWristRServo()
    {
        return rotationalWristServo;
    }
}
