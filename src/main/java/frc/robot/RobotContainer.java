// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.subsystems.Intake.Intake;

public class RobotContainer {
  private final Intake intake;
  private final CommandXboxController driverController;
  public RobotContainer() {
    intake = new Intake();
    driverController = new CommandXboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    driverController.b().whileTrue(new IntakeIn(intake, 0.1));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
