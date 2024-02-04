package frc.robot.SubSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.RobotMap;

public class ShootSubSystem extends SubsystemBase {
    private static CANSparkMax mShootMotor;
    private static RelativeEncoder mShootEncoder;
    private boolean shootEnabled = false;
    private boolean isInverted = false;
    public ShootSubSystem() {
        mShootMotor = new CANSparkMax(RobotMap.SHOOT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        mShootEncoder = mShootMotor.getEncoder();
        shootConfig();
    }

    private void shootConfig() {
        mShootMotor.restoreFactoryDefaults();
        mShootMotor.clearFaults();

        mShootMotor.setSmartCurrentLimit(Constants.SHOOT_CURRENT_LIMIT);
        mShootMotor.setIdleMode(Constants.SHOOT_NEUTRAL_MODE);
        mShootEncoder.setPositionConversionFactor(Constants.SHOOT_GEAR_RATIO);
        mShootEncoder.setVelocityConversionFactor(Constants.SHOOT_GEAR_RATIO / 60.0);

        mShootMotor.setInverted(Constants.SHOOT_INVERTED);

        mShootMotor.getPIDController().setP(Constants.SHOOT_PID[0], 0);
        mShootMotor.getPIDController().setI(Constants.SHOOT_PID[1], 0);
        mShootMotor.getPIDController().setD(Constants.SHOOT_PID[2], 0);
        mShootMotor.getPIDController().setFF(Constants.SHOOT_PID[3], 0);

        mShootMotor.burnFlash();
    }

    public void enableShoot() {
        double rpm = isInverted ? -Constants.SHOOT_SPEED : Constants.SHOOT_SPEED;
        mShootMotor.getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
        shootEnabled = true;
    }
    public void disableableShoot() {
        mShootMotor.stopMotor();
        shootEnabled = false;
    }
    public void reverseShoot() {
        isInverted = !isInverted;
        if (shootEnabled) {
            enableShoot();
        }
    }
}
