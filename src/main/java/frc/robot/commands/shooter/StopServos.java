// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterServos;


public class StopServos extends Command {
  /** Creates a new RotateFeeder. */
  private final ShooterServos c_servos;
  public StopServos(ShooterServos servos) {
    // Use addRequirements() here to declare subsystem dependencies.
    c_servos = servos;
    addRequirements(c_servos);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    c_servos.stopServos();
  }

  public void execute() {
    c_servos.stopServos();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
