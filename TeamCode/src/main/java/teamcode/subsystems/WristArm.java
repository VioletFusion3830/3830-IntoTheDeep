package teamcode.subsystems;

import ftclib.motor.FtcMotorActuator;
import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;

public class WristArm {
    private final TrcMotor armServo;
    private final TrcServo wristRotatorServo;
    private final TrcServo wristVerticalServo;
    private int wristRotatorPosition = 0;

    public WristArm(){
        FtcMotorActuator.Params armParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(RobotParams.ArmParams.PRIMARY_SERVO_NAME,
                        RobotParams.ArmParams.PRIMARY_SERVO_TYPE,
                        RobotParams.ArmParams.PRIMARY_SERVO_INVERTED)
                .setPositionScaleAndOffset(RobotParams.ArmParams.DEGREES_PER_COUNT,
                        RobotParams.ArmParams.POS_OFFSET)
                .setExternalEncoder(RobotParams.ArmParams.EXTERNAL_ENCODER_NAME,
                        RobotParams.ArmParams.EXTERNAL_ENCODER_INVERTED)
                .setPositionPresets(RobotParams.ArmParams.POS_PRESET_TOLERANCE,
                        RobotParams.ArmParams.POS_PRESETS);
        armServo = new FtcMotorActuator(armParams).getMotor();
        armServo.setSoftwarePidEnabled(RobotParams.ArmParams.SOFTWARE_PID_ENABLED);
        armServo.setPositionPidParameters(
                RobotParams.ArmParams.PID_COEFFS,
                RobotParams.ArmParams.PID_TOLERANCE);
        armServo.setPositionPidPowerComp(this::armGetPowerComp);

        FtcServoActuator.Params WristRotatorParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR,
                        RobotParams.WristParamsRotational.PRIMARY_SERVO_ROTATOR_INVERTED);
        wristRotatorServo = new FtcServoActuator(WristRotatorParams).getServo();

        FtcServoActuator.Params WristVerticalParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                        RobotParams.WristParamsVertical.PRIMARY_SERVO_VERTICAL,
                        RobotParams.WristParamsVertical.PRIMARY_SERVO_VERTICAL_INVERTED);
        wristVerticalServo = new FtcServoActuator(WristVerticalParams).getServo();
    }

    public TrcMotor getArmServo()
    {
        return armServo;
    }

    public TrcServo getWristRotationalServo()
    {
        return wristRotatorServo;
    }

    public TrcServo getWristVerticalServo()
    {
        return wristVerticalServo;
    }

    private double armGetPowerComp(double currPower)
    {
        return RobotParams.ArmParams.MAX_GRAVITY_COMP_POWER * Math.cos(Math.toRadians(armServo.getPosition()));
    }   //armGetPowerComp

    /**
     * TO-DO: Check the following four methods for accuracy and optimization.
     * @param positiveCycle if true, cycle up through position presets, otherwise cycle down
     */
    public void cycleRotatorPosition(boolean positiveCycle)
    {
        int posCount = RobotParams.WristParamsRotational.POS_PRESETS.length;
        if (positiveCycle){
            wristRotatorPosition =
                    wristRotatorPosition < posCount - 1 ? wristRotatorPosition + 1 : 0;
        } else{
            wristRotatorPosition =
                    wristRotatorPosition > 0 ? wristRotatorPosition - 1 : posCount - 1;
        }

        setRotatorPosition(wristRotatorPosition);
    }

    /**
     * @param position the index of the position in POS_PRESETS thatthe wrist rotator should be set to
     */
    public void setRotatorPosition(int position){
        wristRotatorServo.setPosition(RobotParams.WristParamsRotational.POS_PRESETS[position]);
    }

    public void cycleVerticalPosition(){ //cycles through position presets for samples
        if (wristVerticalServo.getPosition() == RobotParams.WristParamsVertical.SAMPLE_POSPRESETS[0]){
            setVerticalPosition(1, false);
        } else {
            setVerticalPosition(0, false);
        }
    }

    /**
     * @param isSpecimen if true, only specimen position presets will be cycled through, otherwise sample position presets.
     */
    public void setVerticalPosition(int position, boolean isSpecimen){
        if (isSpecimen){
            wristVerticalServo.setPosition(RobotParams.WristParamsVertical.SPECIMEN_POSPRESETS[position]);
        } else {
            wristVerticalServo.setPosition(RobotParams.WristParamsVertical.SAMPLE_POSPRESETS[position]);
        }
    }

//    public void rotateWrist

}