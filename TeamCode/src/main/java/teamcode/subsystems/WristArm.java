package teamcode.subsystems;

import ftclib.motor.FtcMotorActuator;
import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;

public class WristArm {
    private final TrcMotor armServo;
    private final TrcServo wristRotatorServo;
    private final TrcServo wristVerticalServo;

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
                RobotParams.ArmParams.PID_TOLERANCE);
        armServo.setPositionPidPowerComp(this::armGetPowerComp);

        FtcServoActuator.Params WristRotatorParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR,
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR_INVERTED);
        wristRotatorServo = new FtcServoActuator(WristRotatorParams).getServo();

        FtcServoActuator.Params WristVerticalParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                        RobotParams.WristParamsVertical.PRIMARY_SERVO_VERTICAL,
                        RobotParams.WristParamsVertical.PRIMARY_SERVO_VERTICAL_INVERTED);
        wristVerticalServo = new FtcServoActuator(WristVerticalParams).getServo();
    }

    public TrcMotor getArmServo()
    {
        return armServo;
    }

    public TrcServo getWristRotationalServo()
    {
        return wristRotatorServo;
    }

    public TrcServo getWristVerticalServo()
    {
        return wristVerticalServo;
    }

    public void setWristArmPosition(double armPos,double wristRotationalPos, double wristVerticalPos)
    {
        armServo.setPosition(armPos);
        wristRotatorServo.setPosition(wristRotationalPos);
        wristVerticalServo.setPosition(wristVerticalPos);

    }

    public void setWristVerticalArmPosition(double armPos, double wristVerticalPos)
    {
        armServo.setPosition(armPos);
        wristVerticalServo.setPosition(wristVerticalPos);
    }

    public void setWristVerticalArmSamplePickup()
    {
        setWristVerticalArmPosition(RobotParams.ArmParams.SAMPLE_PICKUP_POS, RobotParams.WristParamsVertical.SAMPLE_PICKUP_POS);
    }

    public void setWristVerticalArmSpecimenPickup()
    {
        setWristVerticalArmPosition(RobotParams.ArmParams.SPECIMEN_PICKUP_POS, RobotParams.WristParamsVertical.SPECIMEN_PICKUP_POS);
    }

    public void setWristVerticalArmSampleDrop()
    {
        setWristVerticalArmPosition(RobotParams.ArmParams.SAMPLE_DROP_POS, RobotParams.WristParamsVertical.SAMPLE_DROP_POS);
    }

    public void setWristVerticalArmSpecimenDrop()
    {
        setWristVerticalArmPosition(RobotParams.ArmParams.SPECIMEN_DROP_P0S, RobotParams.WristParamsVertical.SPECIMEN_DROP_POS);
    }

    private double armGetPowerComp(double currPower)
    {
        return RobotParams.ArmParams.MAX_GRAVITY_COMP_POWER * Math.cos(Math.toRadians(armServo.getPosition()));
    }   //armGetPowerComp

}