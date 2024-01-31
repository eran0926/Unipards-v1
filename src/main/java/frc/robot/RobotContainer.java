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
import frc.robot.SubSystem.SwerveSubSystem;

public class RobotContainer {
  private final SwerveSubSystem swerveSubSystem = new SwerveSubSystem();
  private final XboxController driveController = new XboxController(0);
  // wait until operating system has been designed.
  // private final XboxController operatorController = new XboxController(1);
  public RobotContainer() {
    swerveSubSystem.setDefaultCommand(new SwerveCommands(
      swerveSubSystem,
      () -> driveController.getRawAxis(XboxController.Axis.kLeftY.value),
      () -> driveController.getRawAxis(XboxController.Axis.kLeftX.value),
      () -> driveController.getRawAxis(XboxController.Axis.kRightX.value),
      driveController::getBackButton,
      driveController::getPOV
      
      ));
      
    configureBindings();
  }

  private void configureBindings() {
      new JoystickButton(driveController, XboxController.Button.kRightBumper.value)
        .onTrue(new InstantCommand(swerveSubSystem::zeroGyro));
  }
  public void robotInit(){
    swerveSubSystem.zeroGyro();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}