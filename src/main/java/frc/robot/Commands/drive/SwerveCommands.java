package frc.robot.Commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.Constants;
import frc.robot.SubSystem.SwerveSubSystem;

public class SwerveCommands extends Command {
    private final SwerveSubSystem swerveSubSystem;
    private final DoubleSupplier translationSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;

    private final BooleanSupplier robotCentricSupplier;
    private final IntSupplier povSlowMoveSupplier;

    public SwerveCommands(
        SwerveSubSystem swerveSubSystem,
        DoubleSupplier translationSupplier,
        DoubleSupplier strafeSupplier,
        DoubleSupplier rotationSupplier,
        BooleanSupplier robotCentricSupplier,
        IntSupplier povSlowMoveSupplier
    ){
        this.swerveSubSystem = swerveSubSystem;
        addRequirements(swerveSubSystem);
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSupplier = robotCentricSupplier;
        this.povSlowMoveSupplier = povSlowMoveSupplier;
    }

    @Override
    public void execute(){
        double translationval;
        double strafeval;
        double rotationval = MathUtil.applyDeadband(rotationSupplier.getAsDouble(),Constants.DRIVEJOYSTICK_DEADBAND);
        
        switch (povSlowMoveSupplier.getAsInt()) {
            case 0:
                translationval = Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = 0.0;
                break;
            case 45:
                translationval = Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = -Constants.SWERVE_POV_MOVE_SPEED;
                break;
            case 90:
                translationval = 0.0;
                strafeval = -Constants.SWERVE_POV_MOVE_SPEED;
                break;
            case 135:
                translationval = -Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = -Constants.SWERVE_POV_MOVE_SPEED;
                break;
            case 180:
                translationval = -Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = 0.0;
                break;
            case 225:
                translationval = -Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = Constants.SWERVE_POV_MOVE_SPEED;
                break;
            case 270:
                translationval = 0.0;
                strafeval = Constants.SWERVE_POV_MOVE_SPEED;
                break;
            case 315:
                translationval = Constants.SWERVE_POV_MOVE_SPEED;
                strafeval = Constants.SWERVE_POV_MOVE_SPEED;
                break;
            default:
                translationval = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.DRIVEJOYSTICK_DEADBAND);
                strafeval = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.DRIVEJOYSTICK_DEADBAND);
                break;
        }
        swerveSubSystem.drive(
            new Translation2d(translationval,strafeval).times(Constants.SWERVE_MAX_SPEED),
            rotationval * Constants.SWERVE_MAX_ANGULAR_VELOCITY,
            !robotCentricSupplier.getAsBoolean(),
            true
        );
    }
}
