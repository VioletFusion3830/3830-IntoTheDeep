package teamcode.subsystems;

import trclib.motor.TrcMotor;
import ftclib.motor.FtcMotorActuator;
import teamcode.RobotParams;
import teamcode.Robot;
import trclib.robotcore.TrcDbgTrace;

public class Elevator {
    private final TrcMotor elevator;
    private final Robot robot;

    public Elevator(Robot robot) {
        this.robot = robot;
        FtcMotorActuator.Params elevatorParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(RobotParams.ElevatorParams.PRIMARY_MOTOR_NAME,
                        RobotParams.ElevatorParams.PRIMARY_MOTOR_TYPE,
                        RobotParams.ElevatorParams.PRIMARY_MOTOR_INVERTED)
                .setFollowerMotor(RobotParams.ElevatorParams.FOLLOWER_MOTOR_NAME,
                        RobotParams.ElevatorParams.FOLLOWER_MOTOR_TYPE,
                        RobotParams.ElevatorParams.FOLLOWER_MOTOR_INVERTED)
                .setPositionScaleAndOffset(RobotParams.ElevatorParams.INCHES_PER_COUNT,
                        RobotParams.ElevatorParams.POS_OFFSET)
                .setPositionPresets(RobotParams.ElevatorParams.POS_PRESET_TOLERANCE,
                        RobotParams.ElevatorParams.POS_PRESETS);
        FtcMotorActuator elevatorActuator = new FtcMotorActuator(elevatorParams);
        elevator = elevatorActuator.getMotor();
        TrcMotor elevatorMotor3 = elevatorActuator.createMotor(
                RobotParams.ElevatorParams.SECONDARY_FOLLOWER_MOTOR_NAME,
                RobotParams.ElevatorParams.SECONDARY_FOLLOWER_MOTOR_TYPE,
                null);
        elevatorMotor3.follow(elevator, RobotParams.ElevatorParams.SECONDARY_FOLLOWER_MOTOR_INVERTED);
        //elevator.setTraceLevel(TrcDbgTrace.MsgLevel.DEBUG, true, false, null);
        elevator.setSoftwarePidEnabled(RobotParams.ElevatorParams.SOFTWARE_PID_ENABLED);
        elevator.setPositionPidParameters(
                RobotParams.ElevatorParams.PID_COEFFS,
                RobotParams.ElevatorParams.POS_PID_TOLERANCE);
        elevator.setPositionPidPowerComp(this::getGravityComp);
        elevator.setStallProtection(RobotParams.ElevatorParams.STALL_MIN_POWER,
                RobotParams.ElevatorParams.STALL_TOLERANCE,
                RobotParams.ElevatorParams.STALL_TIMEOUT,
                RobotParams.ElevatorParams.STALL_RESET_TIMEOUT);
    }

    public TrcMotor getElevatorParams()
    {
        return elevator;
    }

    private double getGravityComp(double power)
    {
        double elevatorPos = elevator.getPosition();
        double elbowAngle = robot.elbow.getPosition();
        double distanceToBottom = Math.abs(elevatorPos - RobotParams.ElevatorParams.MIN_POS);

        return distanceToBottom > RobotParams.ElevatorParams.POS_PID_TOLERANCE ? RobotParams.ElevatorParams.MAX_GRAVITY_COMP_POWER*Math.sin(Math.toRadians(elbowAngle)) : 0.0;
    }
} //Class Elevator

