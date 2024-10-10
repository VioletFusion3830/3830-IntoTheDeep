package teamcode.subsystems;

import ftclib.motor.FtcCRServo;
import ftclib.motor.FtcMotorActuator;
import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;

public class Turret {
//Needs some work
    private final TrcMotor turret;

    public Turret(){
        FtcMotorActuator.Params TurretParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(RobotParams.TurretParams.PRIMARY_SERVO_NAME, FtcMotorActuator.MotorType.CRServo,
                        RobotParams.TurretParams.PRIMARY_SERVO_INVERTED)
                .setPositionScaleAndOffset(RobotParams.TurretParams.DEGREES_PER_COUNT,
                        RobotParams.TurretParams.POS_OFFSET)
                .setExternalEncoder(RobotParams.TurretParams.EXTERNAL_ENCODER_NAME,
                        RobotParams.TurretParams.EXTERNAL_ENCODER_INVERTED);
        turret = new FtcMotorActuator(TurretParams).getMotor();
    }

    public TrcMotor getTurretParams()
    {
        return turret;
    }
}
