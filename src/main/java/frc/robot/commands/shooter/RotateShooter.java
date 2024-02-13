// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.ArmRotationEncoder.ArmRotationEncoder;

public class RotateShooter extends Command {
  private final Shooter c_shooter;
  private final double c_ShooterAngle;
  private final double kP = 0;
  private final double kI = 0;
  private final double kD = 0;
  private final PIDController pid;
  private final double TOLERANCE = 0.2;
  private final ArmRotationEncoder c_shooterRotateEncoder;

  public RotateShooter(Shooter shooter, double shooterAngle, ArmRotationEncoder shooterRotateEncoder) {
    c_shooter = shooter;
    c_ShooterAngle = shooterAngle;
    c_shooterRotateEncoder = shooterRotateEncoder;
    pid = new PIDController(kP, kI, kD);
    addRequirements(c_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // pid.calculate(shooterRotateEncoder.get(), target);
  // shooterRotate.set(pid.calculate(shooterRotateEncoder.get(), target));
  // pid.atSetpoint();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = c_shooterRotateEncoder.getShooterEncoderPosition();
    double output = pid.calculate(currentPosition, c_ShooterAngle);
    c_shooter.rotateShooter(output);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_shooter.rotateShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(c_ShooterAngle - c_shooterRotateEncoder.getShooterEncoderPosition()) < TOLERANCE) {
      return true;
    } else {
      return false;
    }

  }
}
