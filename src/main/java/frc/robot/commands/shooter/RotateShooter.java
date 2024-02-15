// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class RotateShooter extends Command {
  private final Shooter c_shooter;
  private final double c_ShooterAngle;

  public RotateShooter(Shooter shooter, double shooterAngle) {
    c_shooter = shooter;
    c_ShooterAngle = shooterAngle;
    addRequirements(c_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    c_shooter.setPIDTarget(c_ShooterAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c_shooter.rotateShooterPID();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_shooter.setRotateVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return c_shooter.pidAtSetpoint();
  }
}
