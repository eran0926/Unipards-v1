package frc.robot.Constants;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.convert.Convertions;

public class Constants {

    public static boolean NAVX_INVERTED = true;
    public static byte NAVX_UPDATE_RATE = 127;
    public static final double NAVX_FUSEDHEADING_OFFSET = -2.50;

    public final static double MAX_VOLTAGE = 12.0;

    public final static Rotation2d FRONT_LEFT_ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
    public final static Rotation2d BACK_LEFT_ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
    public final static Rotation2d BACK_RIGHT_ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
    public final static Rotation2d FRONT_RIGHT_ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);

    //
    public static final double SWERVE_CHASSIS_TRACKWIDTH_METERS = 0.62865;
    public static final double SWERVE_CHASSIS_WHEELBASE_METERS = 0.62865;
    public static final double SWERVE_WHEEL_CIRCUMFERENCE = Convertions.inchesToMeters(4.0) * Math.PI;
    public static final double SWERVE_MAX_SPEED = 4.5; //Wait for test.
    public static final double SWERVE_MAX_ANGULAR_VELOCITY = 4.5;//Wait for test.

    public static final double SWERVE_POV_MOVE_SPEED = 0.2;

    public static final int SWEVRVE_PERIOD_MS = 10;
    public static final double SWERVE_VOLTAGE_COMPENSATION = 12.0;
    public static final int SWERVE_ANGLE_CURRENT_LIMIT = 35;

    public static final double SWERVE_DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final double SWERVE_DRIVE_PEAK_CURRENT_LIMIT = 35;
    public static final double SWERVE_DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean SWERVE_DRIVE_CURRENT_ENABLED = true;

    public static final double SWERVE_DRIVE_PID[] = {0.05, 0.0, 0.0}; // TO DO : Using Tuner.
    public static final double SWERVE_DRIVE_KS = (0.32/MAX_VOLTAGE);
    public static final double SWERVE_DRIVE_KV = (1.51/MAX_VOLTAGE);
    public static final double SWERVE_DRIVE_KA = (0.27/MAX_VOLTAGE);

    public static final double SWERVE_DRIVE_MOTOR_OPENLOOPRAMP = 0.25;
    public static final double SWERVE_DRIVE_MOTOR_CLOSELOOPRAMP = 0.0;
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final CANSparkMax.IdleMode ANGLE_IDLE_MODE = CANSparkMax.IdleMode.kBrake;

    public static final double DRIVEJOYSTICK_DEADBAND = 0.05;

    public static final SwerveDriveKinematics SwerveDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, SWERVE_CHASSIS_WHEELBASE_METERS / 2),
        new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, SWERVE_CHASSIS_WHEELBASE_METERS / 2),
        new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, -SWERVE_CHASSIS_WHEELBASE_METERS / 2),
        new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2, -SWERVE_CHASSIS_WHEELBASE_METERS / 2)
    );

}
