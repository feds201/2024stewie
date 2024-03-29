// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.Leds;

public class SetLEDColor extends Command {
  /** Creates a new SetLEDColor. */
  private final Leds c_leds;
  private final double c_ledColor;
  public SetLEDColor(Leds leds, double ledColor) {
    c_leds = leds;
    c_ledColor = ledColor;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    c_leds.setLedColor(c_ledColor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
