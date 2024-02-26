// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterRotation;

public class RotateShooterBasic extends Command {
  private final ShooterRotation c_shooterRotation;
  private final DoubleSupplier c_voltage;

  public RotateShooterBasic(ShooterRotation shooterRotation, DoubleSupplier voltage) {
    c_shooterRotation = shooterRotation;
    c_voltage = voltage;
    addRequirements(c_shooterRotation);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c_shooterRotation.setRotateVoltage(c_voltage.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_shooterRotation.setRotateVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
