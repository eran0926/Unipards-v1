package frc.robot.SubSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

public class ShootSubSystem extends SubsystemBase {
    private CANSparkMax mShootMotor;
    private RelativeEncoder mShootEncoder;
    public ShootSubSystem() {
        mShootMotor = new CANSparkMax(RobotMap.SHOOT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        mShootEncoder = mShootMotor.getEncoder();
    }

    public void shoot() {
    }
}
