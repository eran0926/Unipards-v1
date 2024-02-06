package frc.robot.SubSystem;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.RobotMap;

public class CollectSubSystem extends SubsystemBase {
  private static CANSparkMax mCollectIntake;
  private static RelativeEncoder mCollectEncoder;
  private boolean intakeEnabled = false;
  public boolean isReverse = false;

  public CollectSubSystem() {
    mCollectIntake = new CANSparkMax(RobotMap.COLLECT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    mCollectEncoder = mCollectIntake.getEncoder();
    collectConfig();
  }

  private void collectConfig() {
    mCollectIntake.restoreFactoryDefaults();

    mCollectIntake.getPIDController().setP(Constants.COLLECT_PID[0], 0);
    mCollectIntake.getPIDController().setI(Constants.COLLECT_PID[1], 0);
    mCollectIntake.getPIDController().setD(Constants.COLLECT_PID[2], 0);
    mCollectIntake.getPIDController().setFF(Constants.COLLECT_PID[3], 0);

    mCollectIntake.setSmartCurrentLimit(Constants.COLLECT_CURRENT_LIMIT);
    mCollectIntake.setIdleMode(Constants.COLLECT_NEUTRAL_MODE);
    mCollectEncoder.setPositionConversionFactor(Constants.COLLECT_GEAR_RATIO);
    mCollectEncoder.setVelocityConversionFactor(Constants.COLLECT_GEAR_RATIO / 60.0);

    mCollectIntake.setInverted(Constants.COLLECT_INVERTED);

    mCollectIntake.burnFlash();
  }

  public void setCollect() {
    double rpm = isReverse ? -Constants.COLLECT_SPEED : Constants.COLLECT_SPEED;
    mCollectIntake.getPIDController().setReference(rpm, CANSparkBase.ControlType.kVelocity);
    intakeEnabled = true;
  }

  public void stopCollect() {
    mCollectIntake.stopMotor();
    intakeEnabled = false;
  }

  public void reverseCollect() {
    isReverse = !isReverse;
    if (intakeEnabled) {
      setCollect();
    }

  }
}
