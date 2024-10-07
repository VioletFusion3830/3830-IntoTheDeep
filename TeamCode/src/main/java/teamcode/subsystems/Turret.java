package teamcode.subsystems;

import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

public class Turret {
//      one servo for something
    private final TrcServo turret;

    public Turret(){
        FtcServoActuator.Params TurretParams = new FtcServoActuator.Params()
                .setPrimaryServo(RobotParams.TurretParams.PRIMARY_SERVO_NAME,
                        RobotParams.TurretParams.PRIMARY_SERVO_INVERTED);

        turret = new FtcServoActuator(TurretParams).getServo();
    }

    public TrcServo getTurretParams()
    {
        return turret;
    }
}
