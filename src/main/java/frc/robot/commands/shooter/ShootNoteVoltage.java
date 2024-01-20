// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShootNoteVoltage extends Command {
  /** Creates a new ShootNote. */
  private final Shooter c_shooter;
  private final DoubleSupplier c_shootVelocity;
  public ShootNoteVoltage(Shooter shooter, DoubleSupplier shootVelocity) {
    c_shooter = shooter;
    c_shootVelocity = shootVelocity;
    addRequirements(c_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c_shooter.rotateShooterWheelsVolts(c_shootVelocity.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_shooter.rotateShooterWheelsVolts(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
