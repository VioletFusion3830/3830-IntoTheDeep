package teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

public class Claw {
    private final TrcServo claw; //one servo for open/close

    public Claw(){
        FtcServoActuator.Params clawParams = new FtcServoActuator.Params()
                .setPrimaryServo(RobotParams.ClawParams.PRIMARY_SERVO_NAME, false)
                .setLogicalPosRange(RobotParams.ClawParams.MIN_POS, RobotParams.ClawParams.MAX_POS)
                .setPhysicalPosRange(RobotParams.ClawParams.MIN_POS, RobotParams.ClawParams.MAX_POS)
                .setMaxStepRate(RobotParams.ClawParams.MAX_STEPRATE)
                .setPositionPresets(RobotParams.ClawParams.POS_PRESET_TOLERANCE, RobotParams.ClawParams.POS_PRESETS);

        claw = new FtcServoActuator(clawParams).getServo();

    }

    public TrcServo getClaw(){return claw;}

}
