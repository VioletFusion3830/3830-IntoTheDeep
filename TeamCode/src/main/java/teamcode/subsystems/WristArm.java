package teamcode.subsystems;

import ftclib.motor.FtcMotorActuator;
import ftclib.motor.FtcServoActuator;
import teamcode.FtcDashboard;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcEvent;

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

    public void setWristArmPosition(double armPos, double wristVerticalPos,TrcEvent event)
    {
        armServo.setPosition(0,armPos,true,RobotParams.ArmParams.POWER_LIMIT,event);
        wristVerticalServo.setPosition(wristVerticalPos);
    }

    public void setWristArmPickupSamplePos(TrcEvent event)
    {
        setWristArmPosition(RobotParams.ArmParams.PICKUP_SAMPLE_POS, RobotParams.WristParamsVertical.PICKUP_SAMPLE_POS,event);
    }

    public void setWristArmPickupSpecimenPos(TrcEvent event)
    {
        setWristArmPosition(RobotParams.ArmParams.PICKUP_SPECIMEN_POS, RobotParams.WristParamsVertical.PICKUP_SPECIMEN_POS, event);
    }

    public void setWristArmBasketScorePos(TrcEvent event)
    {
        setWristArmPosition(RobotParams.ArmParams.BASKET_SCORE_POS, RobotParams.WristParamsVertical.BASKET_SCORE_POS, event);
    }

    public void setWristArmHighChamberScorePos(TrcEvent event)
    {
        setWristArmPosition(RobotParams.ArmParams.HIGH_CHAMBER_SCORE_POS, RobotParams.WristParamsVertical.HIGH_CHAMBER_SCORE_POS, event);
    }

    private double armGetPowerComp(double currPower)
    {
        double armEffectiveAngleInRad = Math.toRadians(robot.elbow.getPosition() + armServo.getPosition());

        return Math.cos(armEffectiveAngleInRad) * FtcDashboard.TunePID.GarvityComp/*RobotParams.ArmParams.MAX_GRAVITY_COMP_POWER*/;
    }   //armGetPowerComp

}