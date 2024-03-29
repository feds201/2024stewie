// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.leds.Leds;

public class BlinkLEDColor extends Command {
  /** Creates a new SetLEDColor. */
  private final Leds c_leds;
  private final double c_ledColorMain;
  private final double c_ledColorSecondary;
  private final double c_time;
  private final double c_repetition;

  public BlinkLEDColor(Leds leds, double ledColorMain, double ledColorSecondary, double time, double repetition) {
    c_leds = leds;
    c_ledColorMain = ledColorMain;
    c_ledColorSecondary = ledColorSecondary;
    c_time = time;
    c_repetition = repetition;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for(int i = 0; i < c_repetition; i++) {
        c_leds.setLedColor(c_ledColorSecondary);
        new WaitCommand(c_time);
        c_leds.setLedColor(c_ledColorMain);
        new WaitCommand(c_time);
    }
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
