// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterWheels;

public class ShootNoteVoltage extends Command {
  /** Creates a new ShootNote. */
  private final ShooterWheels c_shooterWheels;
  private final DoubleSupplier c_shootVelocity;
  public ShootNoteVoltage(ShooterWheels shooterWheels, DoubleSupplier shootVelocity) {
    c_shooterWheels = shooterWheels;
    c_shootVelocity = shootVelocity;
    addRequirements(c_shooterWheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c_shooterWheels.setShootVoltage(c_shootVelocity.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_shooterWheels.setShootVoltage(0.0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
