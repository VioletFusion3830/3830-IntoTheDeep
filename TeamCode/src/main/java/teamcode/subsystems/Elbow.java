package teamcode.subsystems;

import com.qualcomm.robotcore.util.Range;

import ftclib.motor.FtcMotorActuator;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;


public class Elbow
{
    private final Robot robot;
    public final TrcMotor elbow;
    private int elbowPosition;

    public Elbow(Robot robot)
    {
        this.robot = robot;
        FtcMotorActuator.Params elbowParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(
                        RobotParams.ElbowParams.PRIMARY_MOTOR_NAME,
                        RobotParams.ElbowParams.PRIMARY_MOTOR_TYPE,
                        RobotParams.ElbowParams.PRIMARY_MOTOR_INVERTED)
                .setPositionScaleAndOffset(
                        RobotParams.ElbowParams.DEG_PER_COUNT,
                        RobotParams.ElbowParams.POS_OFFSET)
                .setPositionPresets(RobotParams.ElbowParams.POS_PRESET_TOLERANCE,
                        RobotParams.ElbowParams.POS_PRESETS);
        elbow = new FtcMotorActuator(elbowParams).getMotor();
        elbow.setSoftwarePidEnabled(RobotParams.ElbowParams.SOFTWARE_PID_ENABLED);
        elbow.setPositionPidParameters(RobotParams.ElbowParams.PID_COEFFS,
                RobotParams.ElbowParams.PID_TOLERANCE);
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

    private double getElbowPowerComp(double currPower)
    {
        double elevatorPos = robot.elevator.getPosition();
        double elbowAngle = elbow.getPosition();
        double baseValue = 0;

        return RobotParams.ElbowParams.GRAVITY_COMP_MAX_POWER * Math.cos(Math.toRadians(elbowAngle)) * (baseValue+(elevatorPos/RobotParams.ElevatorParams.MAX_POS));
    }   //getElbowPowerComp

}   //class Elbow