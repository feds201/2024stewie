// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.vision.Grab_Note;

public class RobotContainer {
    private final CommandXboxController driverController;

  public RobotContainer() {

    driverController = new CommandXboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    driverController.a().whileTrue(new Grab_Note());
  }

  public Command getAutonomousCommand() {
    return new Grab_Note();
//    return null;
  }
}
