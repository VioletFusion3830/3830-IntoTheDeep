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
    private final Robot robot;

    public WristArm(Robot robot){
        this.robot = robot;
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

    public void setWristArmPosition(String owner, double armDelay, double vWirstDelay, double armPos, double wristVerticalPos,double timeout, TrcEvent event)
    {
        armServo.setPosition(owner,armDelay,armPos,event,timeout);
        verticalWristServo.setPosition(owner, vWirstDelay,wristVerticalPos,null,timeout);
    }

    public void setWristArmPosition(String owner, double armPos, double wristVerticalPos,double timeout, TrcEvent event)
    {
        armServo.setPosition(owner,0,armPos,event,timeout);
        verticalWristServo.setPosition(owner,0,wristVerticalPos,null,timeout);
    }

    public void setWristArmPosition(double armPos, double wristVerticalPos,double timeout)
    {
        armServo.setPosition(0,armPos,null,timeout);
        verticalWristServo.setPosition(0,wristVerticalPos,null,timeout);
    }

    public void setWristArmPickupSamplePos(double timeout, TrcEvent event)
    {
        armServo.setPosition(0,RobotParams.ArmParams.PICKUP_SAMPLE_POS_BASE,event,timeout);
        verticalWristServo.setPosition(0,RobotParams.WristParamsVertical.PICKUP_SAMPLE_POS_BASE,null,timeout);
    }

    public void setWristArmBasketScorePos()
    {
        setWristArmPosition(RobotParams.ArmParams.BASKET_SCORE_POS, RobotParams.WristParamsVertical.BASKET_SCORE_POS, 0);
    }

    //Pickup Specimen Position
    public void setWristArmPickupSpecimenPos(String owner, double armDelay, double vWirstDelay, double timeout, TrcEvent event)
    {
        armServo.setPosition(owner, armDelay, RobotParams.ArmParams.PICKUP_SPECIMEN_POS, event, timeout);
        verticalWristServo.setPosition(owner, vWirstDelay, RobotParams.WristParamsVertical.PICKUP_SPECIMEN_POS,null,timeout);
    }

    public void setWristArmPickupSpecimenPos(String owner, double timeout, TrcEvent event)
    {
        armServo.setPosition(owner,0,RobotParams.ArmParams.PICKUP_SPECIMEN_POS,event,timeout);
        verticalWristServo.setPosition(owner,0,RobotParams.WristParamsVertical.PICKUP_SPECIMEN_POS,null,timeout);
    }

    public void setWristArmPickupSpecimenPos(double timeout, TrcEvent event)
    {
        armServo.setPosition(0,RobotParams.ArmParams.PICKUP_SPECIMEN_POS,event,timeout);
        verticalWristServo.setPosition(0,RobotParams.WristParamsVertical.PICKUP_SPECIMEN_POS,null,timeout);
    }

    //High Chamber Score Position
    public void setWristArmHighChamberScorePos(String owner, double armDelay, double vWirstDelay, double timeout, TrcEvent event)
    {
        armServo.setPosition(owner, armDelay, RobotParams.ArmParams.HIGH_CHAMBER_SCORE_POS, event, timeout);
        verticalWristServo.setPosition(owner, vWirstDelay, RobotParams.WristParamsVertical.HIGH_CHAMBER_SCORE_POS,null,timeout);
    }

    public void setWristArmHighChamberScorePos(String owner, double timeout, TrcEvent event)
    {
        armServo.setPosition(owner,0,RobotParams.ArmParams.HIGH_CHAMBER_SCORE_POS,event,timeout);
        verticalWristServo.setPosition(owner,0,RobotParams.WristParamsVertical.HIGH_CHAMBER_SCORE_POS,null,timeout);
    }

    public void setWristArmHighChamberScorePos(double timeout, TrcEvent event)
    {
        armServo.setPosition(0, RobotParams.ArmParams.HIGH_CHAMBER_SCORE_POS, event, timeout);
        verticalWristServo.setPosition(0, RobotParams.WristParamsVertical.HIGH_CHAMBER_SCORE_POS, null,timeout);
    }
}