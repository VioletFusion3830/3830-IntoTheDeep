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

    public void setWristArmPosition(String owner, double delay, double armPos, double wristVerticalPos,double timeout, TrcEvent event)
    {
        armServo.setPosition(owner,delay,armPos,event,timeout);
        verticalWristServo.setPosition(owner, delay,wristVerticalPos,null,timeout);
    }

    public void setWristArmPosition(String owner, double armPos, double wristVerticalPos,double timeout, TrcEvent event)
    {
        setWristArmPosition(owner,0,armPos,wristVerticalPos,timeout,event);
    }

    public void setWristArmPosition(double armPos, double wristVerticalPos)
    {
        setWristArmPosition(null,armPos,wristVerticalPos,0,null);
    }

    //Pickup Sample Position
    public void setWristArmPickupSamplePos(String owner,double timeout, TrcEvent event)
    {
        setWristArmPosition(owner,robot.armPickupSamplePos(),robot.verticalWristPickupSamplePos(),timeout,event);
    }

    public void setWristArmPickupSamplePos()
    {
        setWristArmPosition(robot.armPickupSamplePos(),robot.verticalWristPickupSamplePos());
    }

    //Ready pickup Sample Position
    public void setWristArmPickupReadySamplePos(String owner,double timeout, TrcEvent event)
    {
        setWristArmPosition(owner,robot.armReadySamplePickupPos(),robot.verticalWristReadySamplePickupPos(),timeout,event);
    }

    public void setWristArmPickupReadySamplePos()
    {
        setWristArmPosition(robot.armReadySamplePickupPos(),robot.verticalWristReadySamplePickupPos());
    }

    //Basket Score Position
    public void setWristArmBasketScorePos(String owner,double timeout, TrcEvent event)
    {
        setWristArmPosition(owner,RobotParams.ArmParams.BASKET_SCORE_POS, RobotParams.WristParamsVertical.BASKET_SCORE_POS,timeout,event);
    }

    public void setWristArmBasketScorePos()
    {
        setWristArmPosition(RobotParams.ArmParams.BASKET_SCORE_POS, RobotParams.WristParamsVertical.BASKET_SCORE_POS);
    }

    //Pickup Specimen Position

    public void setWristArmPickupSpecimenPos(String owner, double timeout, TrcEvent event)
    {
        setWristArmPosition(owner,RobotParams.ArmParams.PICKUP_SPECIMEN_POS,RobotParams.WristParamsVertical.PICKUP_SPECIMEN_POS,timeout,event);
    }

    public void setWristArmPickupSpecimenPos()
    {
        setWristArmPosition(RobotParams.ArmParams.PICKUP_SPECIMEN_POS,RobotParams.WristParamsVertical.PICKUP_SPECIMEN_POS);
    }

    //High Chamber Score Position
    public void setWristArmHighChamberScorePos(String owner, double timeout, TrcEvent event)
    {
        setWristArmPosition(owner,RobotParams.ArmParams.HIGH_CHAMBER_SCORE_POS,RobotParams.WristParamsVertical.HIGH_CHAMBER_SCORE_POS,timeout,event);
    }

    public void setWristArmHighChamberScorePos()
    {
        setWristArmPosition(RobotParams.ArmParams.HIGH_CHAMBER_SCORE_POS,RobotParams.WristParamsVertical.HIGH_CHAMBER_SCORE_POS);
    }
}