package frc.robot.SubSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.RobotMap;

public class ArmSubSystem extends SubsystemBase {
    private CANSparkMax mArmMotorLeft;
    private CANSparkMax mArmMotorRight;
    private RelativeEncoder mArmEncoderLeft;
    private RelativeEncoder mArmEncoderRight;


    public ArmSubSystem() {
        mArmMotorLeft = new CANSparkMax(RobotMap.ARM_LEFT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        mArmMotorRight = new CANSparkMax(RobotMap.ARM_RIGHT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        mArmEncoderLeft = mArmMotorLeft.getEncoder();
        mArmEncoderRight = mArmMotorRight.getEncoder();
        armConfig(mArmMotorLeft, mArmEncoderLeft);
        armConfig(mArmMotorRight, mArmEncoderRight);


    }

    private void armConfig(CANSparkMax mArmMotor, RelativeEncoder mArmMotorEncoder){
        mArmMotor.restoreFactoryDefaults();
        mArmMotor.getPIDController().setP(Constants.ARM_PID[0], 0);
        mArmMotor.getPIDController().setI(Constants.ARM_PID[1], 0);
        mArmMotor.getPIDController().setD(Constants.ARM_PID[2], 0);
        mArmMotor.getPIDController().setFF(Constants.ARM_PID[3], 0);
        mArmMotor.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT);
        mArmMotor.setIdleMode(Constants.ARM_NEUTRAL_MODE);
        mArmMotorEncoder.setPositionConversionFactor(Constants.ARM_GEAR_RATIO);
        mArmMotorEncoder.setVelocityConversionFactor(Constants.ARM_GEAR_RATIO / 60.0);
        mArmMotor.burnFlash();
    }

    public void setArm(int setPoint) {
        mArmMotorLeft.getPIDController().setReference(setPoint, CANSparkMax.ControlType.kPosition);
        mArmMotorRight.getPIDController().setReference(setPoint, CANSparkMax.ControlType.kPosition);
    }
}
