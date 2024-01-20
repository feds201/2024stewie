// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
  private final Shooter shooter;
  private final CommandXboxController driverController;
  public RobotContainer() {
    shooter = new Shooter();
    driverController = new CommandXboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    driverController.a().whileTrue(new ShootNote(shooter, () -> shooter.getShooterSpeed()));
  }

  public Command getAutonomousCommand() { 
    return Commands.print("No autonomous command configured");
  }
}
