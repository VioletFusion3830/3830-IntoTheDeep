package teamcode.subsystems;

import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

public class Wrist {

    private final TrcServo clawVertical;
    private final TrcServo clawRotator;

    FtcServoActuator.Params clawVerticalParams = new FtcServoActuator.Params(
            .setPrimaryServo(RobotParams.WristParams.SUBSYSTEM_NAME, RobotParams.WristParams.PRIMARY_SERVO_INVERTED)
            .

    );

    FtcServoActuator.Params clawRotatorParams = new FtcServoActuator.Params(

    );

    public TrcServo getClawVertical() {return clawVertical;}
    public TrcServo getClawRotator() {return clawRotator;}

}
