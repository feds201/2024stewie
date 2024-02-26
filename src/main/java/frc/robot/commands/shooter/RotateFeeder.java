// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterFeeder;


public class RotateFeeder extends Command {
  /** Creates a new RotateFeeder. */
  private final ShooterFeeder c_ShooterFeeder;
  private final DoubleSupplier c_speed;
  public RotateFeeder(ShooterFeeder shooterFeeder, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    c_ShooterFeeder = shooterFeeder;
    c_speed = speed;
    addRequirements(c_ShooterFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c_ShooterFeeder.setCurrentServoSpeed(c_speed.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     c_ShooterFeeder.setCurrentServoSpeed(0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
