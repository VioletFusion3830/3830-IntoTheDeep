package teamcode.subsystems;

import ftclib.motor.FtcServoActuator;
import teamcode.FtcDashboard;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcEvent;

public class WristArm {
    private final TrcServo armServo;
    private final TrcServo verticalWristServo;

    public WristArm(Robot robot){
        FtcServoActuator.Params armParams = new FtcServoActuator.Params()
                .setPrimaryServo(RobotParams.ArmParams.PRIMARY_SERVO_NAME,RobotParams.ArmParams.PRIMARY_SERVO_INVERTED);
        armServo = new FtcServoActuator(armParams).getServo();

        FtcServoActuator.Params vWristParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                        RobotParams.WristParamsVertical.PRIMARY_SERVO_VERTICAL,
                        RobotParams.WristParamsVertical.PRIMARY_SERVO_VERTICAL_INVERTED);
        verticalWristServo = new FtcServoActuator(vWristParams).getServo();
    }

    public TrcServo getArmServo()
    {
        return armServo;
    }

    public TrcServo getWristVerticalServo()
    {
        return verticalWristServo;
    }

    public void setWristArmPosition(double armPos, double wristVerticalPos,double timeout)
    {
        armServo.setPosition(armPos,null,timeout);
        verticalWristServo.setPosition(wristVerticalPos);
    }

    public void setWristArmPickupSamplePos(double timeout)
    {
        //setWristArmPosition(RobotParams.ArmParams.PICKUP_SAMPLE_POS, RobotParams.WristParamsVertical.PICKUP_SAMPLE_POS,timeout);
    }

    public void setWristArmPickupSpecimenPos(double timeout)
    {
        setWristArmPosition(RobotParams.ArmParams.PICKUP_SPECIMEN_POS,RobotParams.WristParamsVertical.PICKUP_SPECIMEN_POS, timeout);
    }

    public void setWristArmBasketScorePos(double timeout)
    {
        setWristArmPosition(RobotParams.ArmParams.BASKET_SCORE_POS, RobotParams.WristParamsVertical.BASKET_SCORE_POS, timeout);
    }

    public void setWristArmHighChamberScorePos(double timeout)
    {
        setWristArmPosition(RobotParams.ArmParams.HIGH_CHAMBER_SCORE_POS, RobotParams.WristParamsVertical.HIGH_CHAMBER_SCORE_POS, timeout);
    }
}