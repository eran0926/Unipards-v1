// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.drive.SwerveCommands;
import frc.robot.SubSystem.ArmSubSystem;
import frc.robot.SubSystem.CollectSubSystem;
import frc.robot.SubSystem.ShootSubSystem;
import frc.robot.SubSystem.SwerveSubSystem;

public class RobotContainer {
  private final XboxController driveController = new XboxController(0);
  // wait until operating system has been designed.
   private final XboxController operatorController = new XboxController(1);
  private final SwerveSubSystem swerveSubSystem = new SwerveSubSystem();
  private final CollectSubSystem collectSubSystem = new CollectSubSystem();
  private final ShootSubSystem shootSubSystem = new ShootSubSystem();
  private final ArmSubSystem armSubSystem = new ArmSubSystem();
  public RobotContainer() {
    swerveSubSystem.setDefaultCommand(new SwerveCommands(
      swerveSubSystem,
      () -> driveController.getRawAxis(XboxController.Axis.kLeftY.value),
      () -> driveController.getRawAxis(XboxController.Axis.kLeftX.value),
      () -> driveController.getRawAxis(XboxController.Axis.kRightX.value),
      driveController::getLeftBumper,
      driveController::getPOV
      
      ));
      
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driveController, XboxController.Button.kRightBumper.value)
                    .onTrue(new InstantCommand(swerveSubSystem::zeroGyro));

//    Bind intake button
    new JoystickButton(operatorController, XboxController.Button.kA.value)
            .onTrue(new InstantCommand(collectSubSystem::setCollect));
    new JoystickButton(operatorController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(collectSubSystem::stopCollect));
    new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
            .onTrue(new InstantCommand(collectSubSystem::reverseCollect));

//    Bind shoot button
    new JoystickButton(operatorController, XboxController.Button.kX.value)
            .onTrue(new InstantCommand(shootSubSystem::enableShoot));
    new JoystickButton(operatorController, XboxController.Button.kY.value)
            .onTrue(new InstantCommand(shootSubSystem::disableableShoot));
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
            .onTrue(new InstantCommand(shootSubSystem::reverseShoot));

    new JoystickButton(operatorController, XboxController.Button.kStart.value)
            .onTrue(new InstantCommand(armSubSystem::toCollectPosition));
    new JoystickButton(operatorController, XboxController.Button.kBack.value)
            .onTrue(new InstantCommand(armSubSystem::toSpeakerPosition
            ));

  }
  public void robotInit(){
    //swerveSubSystem.zeroGyro();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
