package teamcode.subsystems;

import ftclib.motor.FtcMotorActuator;
import ftclib.motor.FtcServoActuator;
import teamcode.FtcDashboard;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;

public class WristArm {
    private final TrcMotor armServo;
    private final Robot robot;
    private final TrcServo wristVerticalServo;

    public WristArm(Robot robot){
        this.robot = robot;
        FtcMotorActuator.Params armParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(RobotParams.ArmParams.PRIMARY_SERVO_NAME,
                        RobotParams.ArmParams.PRIMARY_SERVO_TYPE,
                        RobotParams.ArmParams.PRIMARY_SERVO_INVERTED)
                .setPositionScaleAndOffset(RobotParams.ArmParams.ARM_DEGREE_SCALE,
                        RobotParams.ArmParams.POS_OFFSET,RobotParams.ArmParams.ZERO_OFFSET)
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

    public TrcServo getWristVerticalServo()
    {
        return wristVerticalServo;
    }

    public void setWristArmPosition(double armPos, double wristVerticalPos)
    {
        armServo.setPosition(armPos);
        wristVerticalServo.setPosition(wristVerticalPos);
    }

    public void setWristArmPickupSamplePos()
    {
        setWristArmPosition(RobotParams.ArmParams.PICKUP_SAMPLE_POS, RobotParams.WristParamsVertical.PICKUP_SAMPLE_POS);
    }

    public void setWristArmPickupSpecimenPos()
    {
        setWristArmPosition(RobotParams.ArmParams.PICKUP_SPECIMEN_POS, RobotParams.WristParamsVertical.PICKUP_SPECIMEN_POS);
    }

    public void setWristArmBasketScorePos()
    {
        setWristArmPosition(RobotParams.ArmParams.BASKET_SCORE_POS, RobotParams.WristParamsVertical.BASKET_SCORE_POS);
    }

    public void setWristArmHighChamberScorePos()
    {
        setWristArmPosition(RobotParams.ArmParams.HIGH_CHAMBER_SCORE_POS, RobotParams.WristParamsVertical.HIGH_CHAMBER_SCORE_POS);
    }

    private double armGetPowerComp(double currPower)
    {
        double armEffectiveAngleInRad = Math.toRadians(robot.elbow.getPosition() + armServo.getPosition());

        return Math.cos(armEffectiveAngleInRad) * FtcDashboard.TunePID.GarvityComp/*RobotParams.ArmParams.MAX_GRAVITY_COMP_POWER*/;
    }   //armGetPowerComp

}