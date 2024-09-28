package teamcode.subsystems;

import trclib.motor.TrcMotor;
import ftclib.motor.FtcMotorActuator;
import teamcode.RobotParams;

public class Elevator {
    private final TrcMotor elevator;

    public Elevator() {
        FtcMotorActuator.Params elevatorParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(RobotParams.Elevator.PRIMARY_MOTOR_NAME, RobotParams.Elevator.PRIMARY_MOTOR_TYPE, RobotParams.Elevator.PRIMARY_MOTOR_INVERTED)
                .setFollowerMotor(RobotParams.Elevator.FOLLOWER_MOTOR_NAME, RobotParams.Elevator.FOLLOWER_MOTOR_TYPE,RobotParams.Elevator.FOLLOWER_MOTOR_INVERTED)
                .setPositionScaleAndOffset(RobotParams.Elevator.INCHES_PER_COUNT, RobotParams.Elevator.POS_OFFSET)
                .setPositionPresets(RobotParams.Elevator.POS_PRESET_TOLERANCE,RobotParams.Elevator.posPresets);
        elevator = new FtcMotorActuator(elevatorParams).getMotor();
        elevator.setSoftwarePidEnabled(RobotParams.Elevator.SOFTWARE_PID_ENABLED);
        elevator.setPositionPidParameters(
                RobotParams.Elevator.posPIDCoeffs, RobotParams.Elevator.POS_PID_TOLERANCE);
        elevator.setPositionPidPowerComp(this::getGravityComp);
        elevator.setStallProtection(
                RobotParams.Elevator.STALL_MIN_POWER, RobotParams.Elevator.STALL_TOLERANCE,
                RobotParams.Elevator.STALL_TIMEOUT, RobotParams.Elevator.STALL_RESET_TIMEOUT);
    }

    public TrcMotor getElevator()
    {
        return elevator;
    }

    private double getGravityComp(double power)
    {
        double elevatorPos = elevator.getPosition();
//        double distanceToTop = Math.abs(RobotParams.Elevator.MAX_POS - elevatorPos);
        double distanceToBottom = Math.abs(elevatorPos - RobotParams.Elevator.MIN_POS);
        double powerComp = distanceToBottom > RobotParams.Elevator.POS_PID_TOLERANCE ? RobotParams.Elevator.GRAVITY_COMP_POWER: 0.0;

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
}

