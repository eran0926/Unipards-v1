package frc.robot.SubSystem;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.RobotMap;

import static com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle;

public class ArmSubSystem extends SubsystemBase {
    private CANSparkMax mArmMotorFirstLeft;
    private CANSparkMax mArmMotorFirstRight;
    private CANSparkMax mArmMotorSecond;
    private AbsoluteEncoder firstArmAbsluteEncoder;
    private AbsoluteEncoder secondArmAbsluteEncoder;
    private RelativeEncoder mArmEncoderLeft;
    private RelativeEncoder mArmEncoderRight;
    private RelativeEncoder mArmEncoderSecond;
    private double firstArmSetPoint;
    private double secondArmSetPoint;


    public ArmSubSystem() {
        mArmMotorFirstLeft = new CANSparkMax(RobotMap.FIRST_ARM_LEFT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        mArmMotorFirstRight = new CANSparkMax(RobotMap.FIRST_ARM_RIGHT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        mArmMotorSecond = new CANSparkMax(RobotMap.SECOND_ARM_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        mArmEncoderLeft = mArmMotorFirstLeft.getEncoder();
        mArmEncoderRight = mArmMotorFirstRight.getEncoder();
//        firstArmAbsluteEncoder = mArmMotorFirstRight.getAbsoluteEncoder(kDutyCycle);
        firstArmAbsluteEncoder = mArmMotorFirstRight.getAbsoluteEncoder(kDutyCycle);
        secondArmAbsluteEncoder = mArmMotorSecond.getAbsoluteEncoder(kDutyCycle);
        mArmEncoderSecond = mArmMotorSecond.getEncoder();
        firstArmConfig(mArmMotorFirstLeft, mArmEncoderLeft, false);
        firstArmConfig(mArmMotorFirstRight, mArmEncoderRight, true);
        SecondArmConfig(mArmMotorSecond, mArmEncoderSecond, false);


    }

    private void firstArmConfig(CANSparkMax mArmMotor, RelativeEncoder mArmMotorEncoder, boolean Inverted){
        mArmMotor.restoreFactoryDefaults();
        mArmMotor.getPIDController().setP(Constants.ARM_CONSTANTS.FIRST_ARM_PID[0], 0);
        mArmMotor.getPIDController().setI(Constants.ARM_CONSTANTS.FIRST_ARM_PID[1], 0);
        mArmMotor.getPIDController().setD(Constants.ARM_CONSTANTS.FIRST_ARM_PID[2], 0);
        mArmMotor.getPIDController().setFF(Constants.ARM_CONSTANTS.FIRST_ARM_PID[3], 0);
        mArmMotor.setSmartCurrentLimit(Constants.ARM_CONSTANTS.ARM_CURRENT_LIMIT);
//        TODO:fix soft limit
        mArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.ARM_FORWARD_SOFT_LIMIT);
        mArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.ARM_REVERSE_SOFT_LIMIT);
        mArmMotor.setIdleMode(Constants.ARM_CONSTANTS.FIRST_ARM_IDLE_MODE);
        mArmMotor.setInverted(Inverted);
        mArmMotorEncoder.setPosition(0);
//        mArmMotorEncoder.setPositionConversionFactor(Constants.ARM_GEAR_RATIO);
//        mArmMotorEncoder.setVelocityConversionFactor(Constants.ARM_GEAR_RATIO / 60.0);
        mArmMotor.burnFlash();
    }private void SecondArmConfig(CANSparkMax mArmMotor, RelativeEncoder mArmMotorEncoder, boolean Inverted){
        mArmMotor.restoreFactoryDefaults();
        mArmMotor.getPIDController().setP(Constants.ARM_CONSTANTS.SECOND_ARM_PID[0], 0);
        mArmMotor.getPIDController().setI(Constants.ARM_CONSTANTS.SECOND_ARM_PID[1], 0);
        mArmMotor.getPIDController().setD(Constants.ARM_CONSTANTS.SECOND_ARM_PID[2], 0);
        mArmMotor.getPIDController().setFF(Constants.ARM_CONSTANTS.SECOND_ARM_PID[3], 0);
        mArmMotor.setSmartCurrentLimit(Constants.ARM_CONSTANTS.ARM_CURRENT_LIMIT);
//        TODO:fix soft limit
        mArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.ARM_FORWARD_SOFT_LIMIT);
        mArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.ARM_REVERSE_SOFT_LIMIT);
        mArmMotor.setIdleMode(Constants.ARM_CONSTANTS.SECOND_ARM_IDLE_MODE);
        mArmMotor.setInverted(Inverted);
        mArmMotorEncoder.setPosition(0);
        mArmMotorEncoder.setPositionConversionFactor(Constants.ARM_CONSTANTS.SECOND_ARM_GEAR_RATIO);
        mArmMotorEncoder.setVelocityConversionFactor(Constants.ARM_CONSTANTS.SECOND_ARM_GEAR_RATIO / 60.0);
        mArmMotor.burnFlash();
    }

    public void setArm(double firstArmSetPoint, double secondArmSetPoint) {
        this.firstArmSetPoint = firstArmSetPoint;
        this.secondArmSetPoint = secondArmSetPoint;
        SmartDashboard.putNumber("fArmReference", firstArmSetPoint);
        SmartDashboard.putNumber("sArmReference", secondArmSetPoint);
        mArmMotorFirstLeft.getPIDController().setReference(firstArmSetPoint, CANSparkMax.ControlType.kPosition);
        mArmMotorFirstRight.getPIDController().setReference(firstArmSetPoint, CANSparkMax.ControlType.kPosition);
        mArmMotorSecond.getPIDController().setReference(secondArmSetPoint, CANSparkMax.ControlType.kPosition);
    }

    public void AddSecondArmPosition(){
        setArm(this.firstArmSetPoint, this.secondArmSetPoint + 0.1);
    }
    public void SubSecondArmPosition(){
        setArm(this.firstArmSetPoint, this.secondArmSetPoint - 0.1);
    }
//    public void VOID(){
//        setArm(this.firstArmSetPoint, this.secondArmSetPoint);
//    }

//    public void toLowest() {
//        position = Constants.ARM_POSITIONS.get(Constants.ArmPosition.LOWEST);
//        setArm(position);
//    }
//
//    public void toCollectPosition() {
//        position = Constants.ARM_POSITIONS.get(Constants.ArmPosition.COLLECT);
//        setArm(position);
//    }
//    public void toAmpPosition() {
//        position = Constants.ARM_POSITIONS.get(Constants.ArmPosition.AMP);
//        setArm(position);
//    }
//    public void toSpeakerPosition() {
//        position = Constants.ARM_POSITIONS.get(Constants.ArmPosition.SPEAKER);
//        setArm(position);
//    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Arm Position", mArmEncoderLeft.getPosition());
        SmartDashboard.putNumber("Right Arm Position", mArmEncoderRight.getPosition());
        SmartDashboard.putNumber("S Arm Position", mArmEncoderSecond.getPosition());
        SmartDashboard.putNumber("f Arm abs Position", firstArmAbsluteEncoder.getPosition());
        SmartDashboard.putNumber("s Arm abs Position", secondArmAbsluteEncoder.getPosition());
    }
}
