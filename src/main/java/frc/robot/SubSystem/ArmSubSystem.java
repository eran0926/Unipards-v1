package frc.robot.SubSystem;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.RobotMap;

public class ArmSubSystem extends SubsystemBase {
    private CANSparkMax mArmMotorLeft;
    private CANSparkMax mArmMotorRight;
    private RelativeEncoder mArmEncoderLeft;
    private RelativeEncoder mArmEncoderRight;

    private double position;


    public ArmSubSystem() {
        mArmMotorLeft = new CANSparkMax(RobotMap.ARM_LEFT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        mArmMotorRight = new CANSparkMax(RobotMap.ARM_RIGHT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        mArmEncoderLeft = mArmMotorLeft.getEncoder();
        mArmEncoderRight = mArmMotorRight.getEncoder();
        armConfig(mArmMotorLeft, mArmEncoderLeft, false);
        armConfig(mArmMotorRight, mArmEncoderRight, true);


    }

    private void armConfig(CANSparkMax mArmMotor, RelativeEncoder mArmMotorEncoder, boolean Inverted){
        mArmMotor.restoreFactoryDefaults();
        mArmMotor.getPIDController().setP(Constants.ARM_PID[0], 0);
        mArmMotor.getPIDController().setI(Constants.ARM_PID[1], 0);
        mArmMotor.getPIDController().setD(Constants.ARM_PID[2], 0);
        mArmMotor.getPIDController().setFF(Constants.ARM_PID[3], 0);
        mArmMotor.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT);
//        TODO:fix soft limit
        mArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.ARM_FORWARD_SOFT_LIMIT);
        mArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.ARM_REVERSE_SOFT_LIMIT);
        mArmMotor.setIdleMode(Constants.ARM_NEUTRAL_MODE);
        mArmMotor.setInverted(Inverted);
        mArmMotorEncoder.setPosition(0);
//        mArmMotorEncoder.setPositionConversionFactor(Constants.ARM_GEAR_RATIO);
//        mArmMotorEncoder.setVelocityConversionFactor(Constants.ARM_GEAR_RATIO / 60.0);
        mArmMotor.burnFlash();
    }

    public void setArm(double setPoint) {
        SmartDashboard.putNumber("ArmRefference", setPoint);
        mArmMotorLeft.getPIDController().setReference(setPoint, CANSparkMax.ControlType.kPosition);
        mArmMotorRight.getPIDController().setReference(setPoint, CANSparkMax.ControlType.kPosition);
    }

    public void toLowest() {
        position = Constants.ARM_POSITIONS.get(Constants.ArmPosition.LOWEST);
        setArm(position);
    }

    public void toCollectPosition() {
        position = Constants.ARM_POSITIONS.get(Constants.ArmPosition.COLLECT);
        setArm(position);
    }
    public void toAmpPosition() {
        position = Constants.ARM_POSITIONS.get(Constants.ArmPosition.AMP);
        setArm(position);
    }
    public void toSpeakerPosition() {
        position = Constants.ARM_POSITIONS.get(Constants.ArmPosition.SPEAKER);
        setArm(position);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Arm Position", mArmEncoderLeft.getPosition());
        SmartDashboard.putNumber("Right Arm Position", mArmEncoderRight.getPosition());
    }
}
