package teamcode.subsystems;

import ftclib.motor.FtcMotorActuator;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;


public class Elbow
{
    public final TrcMotor elbow;

    public Elbow()
    {
        FtcMotorActuator.Params elbowParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(
                        RobotParams.ElbowParams.PRIMARY_MOTOR_NAME,
                        RobotParams.ElbowParams.PRIMARY_MOTOR_TYPE,
                        RobotParams.ElbowParams.PRIMARY_MOTOR_INVERTED)
                .setPositionScaleAndOffset(
                        RobotParams.ElbowParams.DEG_SCALE,
                        RobotParams.ElbowParams.POS_OFFSET,
                        RobotParams.ElbowParams.ZERO_OFFSET)
                .setPositionPresets(RobotParams.ElbowParams.POS_PRESET_TOLERANCE,
                        RobotParams.ElbowParams.posPresets);
        elbow = new FtcMotorActuator(elbowParams).getMotor();
        elbow.setPositionPidParameters(
                RobotParams.ElbowParams.posPidCoeffs,
                RobotParams.ElbowParams.POS_PID_TOLERANCE);
        elbow.setPositionPidPowerComp(this::getElbowPowerComp);
        elbow.setPidStallDetectionEnabled(
                RobotParams.ElbowParams.STALL_RESET_TIMEOUT,
                RobotParams.ElbowParams.STALL_TIMEOUT,
                RobotParams.ElbowParams.STALL_TOLERANCE);
    }   //Elbow

    public TrcMotor getElbow()
    {
        return elbow;
    }   //getMotor


    private double getElbowPowerComp(double currPower)
    {
        return RobotParams.ElbowParams.GRAVITY_COMP_MAX_POWER * Math.sin(Math.toRadians(elbow.getPosition()));
    }   //getElbowPowerComp

}   //class Elbow