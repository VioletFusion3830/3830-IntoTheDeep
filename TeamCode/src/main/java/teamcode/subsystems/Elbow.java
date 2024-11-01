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
        elbow.setSoftwarePidEnabled(RobotParams.ElbowParams.SOFTWARE_PID_ENABLED);
        elbow.setPositionPidParameters(RobotParams.ElbowParams.PID_COEFFS,
                RobotParams.ElbowParams.PID_TOLERANCE);
        elbow.setPositionPidParameters(
                RobotParams.ElbowParams.POS_PID_COEFFS,
                RobotParams.ElbowParams.POS_PID_TOLERANCE);
        elbow.setPositionPidPowerComp(this::getElbowPowerComp);
        elbow.setStallProtection(RobotParams.ElbowParams.STALL_MIN_POWER,
                RobotParams.ElbowParams.STALL_TOLERANCE,
                RobotParams.ElbowParams.STALL_TIMEOUT,
                RobotParams.ElbowParams.STALL_RESET_TIMEOUT);
    }   //Elbow

    public TrcMotor getElbow()
    {
        return elbow;
    }   //getMotor

    //May need to make it so gravity comp also takes in the elevators extension amount
    private double getElbowPowerComp(double currPower)
    {
        return RobotParams.ElbowParams.GRAVITY_COMP_MAX_POWER * Math.sin(Math.toRadians(elbow.getPosition()));
    }   //getElbowPowerComp

}   //class Elbow