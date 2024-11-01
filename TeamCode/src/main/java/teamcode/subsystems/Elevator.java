package teamcode.subsystems;

import trclib.motor.TrcMotor;
import ftclib.motor.FtcMotorActuator;
import teamcode.RobotParams;

public class Elevator {
    private final TrcMotor elevator;

    public Elevator() {
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
        elevator = new FtcMotorActuator(elevatorParams).getMotor();
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
//        double distanceToTop = Math.abs(RobotParams.ElevatorParams.MAX_POS - elevatorPos);
        double distanceToBottom = Math.abs(elevatorPos - RobotParams.ElevatorParams.MIN_POS);
        double powerComp = distanceToBottom > RobotParams.ElevatorParams.POS_PID_TOLERANCE ? RobotParams.ElevatorParams.GRAVITY_COMP_POWER: 0.0;

//        if(power < 0.0) {
//            if (distanceToBottom <= RobotParams.ELEVATOR_POWERCOMP_DISTANCE_THRESHOLD_LOWER) {
//                powerComp += power * (RobotParams.ELEVATOR_SLOWDOWN_SCALE_LOWER - 1.0);
//            }
//        }
//        else
//        {
//            if (distanceToTop <= RobotParams.ELEVATOR_POWERCOMP_DISTANCE_THRESHOLD_UPPER) {
//                powerComp = power * (RobotParams.ELEVATOR_SLOWDOWN_SCALE_UPPER - 1.0);
//            }
//        }
        return powerComp;
    }
} //Class Elevator

