package teamcode.subsystems;

import ftclib.motor.FtcMotorActuator;
import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;

public class WristArm {
    private final TrcMotor armServo;
    private final TrcServo wristRotator;
    private final TrcServo wristVertical;

    public WristArm(){
        FtcMotorActuator.Params armParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(RobotParams.ArmParams.PRIMARY_SERVO_NAME,
                        RobotParams.ArmParams.PRIMARY_SERVO_TYPE,
                        RobotParams.ArmParams.PRIMARY_SERVO_INVERTED)
                .setPositionScaleAndOffset(RobotParams.ArmParams.DEGREES_PER_COUNT,
                        RobotParams.ArmParams.POS_OFFSET)
                .setExternalEncoder(RobotParams.ArmParams.EXTERNAL_ENCODER_NAME,
                        RobotParams.ArmParams.EXTERNAL_ENCODER_INVERTED)
                .setPositionPresets(RobotParams.ArmParams.POS_PRESET_TOLERANCE,
                        RobotParams.ArmParams.POS_PRESETS);
        armServo = new FtcMotorActuator(armParams).getMotor();
        armServo.setSoftwarePidEnabled(RobotParams.ArmParams.SOFTWARE_PID_ENABLED);
        armServo.setPositionPidParameters(
                RobotParams.ArmParams.PID_COEFFS,
                RobotParams.ArmParams.POS_PID_TOLERANCE);
        armServo.setPositionPidPowerComp(this::armGetPowerComp);
        armServo.setStallProtection(RobotParams.ArmParams.STALL_MIN_POWER,
                RobotParams.ArmParams.STALL_TOLERANCE,
                RobotParams.ArmParams.STALL_TIMEOUT,
                RobotParams.ArmParams.STALL_RESET_TIMEOUT);

        FtcServoActuator.Params WristRotatorParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR,
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR_INVERTED);
        wristRotator = new FtcServoActuator(WristRotatorParams).getServo();

        FtcServoActuator.Params WristVerticalParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                        RobotParams.WristParamsVertical.PRIMARY_SERVO_VERTICAL,
                        RobotParams.WristParamsVertical.PRIMARY_SERVO_VERTICAL_INVERTED);
        wristVertical = new FtcServoActuator(WristVerticalParams).getServo();
    }

    public TrcMotor getArmServo()
    {
        return armServo;
    }

    public TrcServo getWristRotational()
    {
        return wristRotator;
    }

    public TrcServo getWristVertical()
    {
        return wristVertical;
    }

    private double armGetPowerComp(double currPower)
    {
        return RobotParams.ArmParams.MAX_GRAVITY_COMP_POWER * Math.cos(Math.toRadians(armServo.getPosition()));
    }   //armGetPowerComp



}