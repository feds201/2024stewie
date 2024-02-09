// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRotationEncoder.ArmRotationEncoder;
import frc.robot.subsystems.Intake.Intake;

public class RotateArm extends Command {
  /** Creates a new RotateArm. */
  private final Intake c_intake;
  private final double c_angle;
  private final PIDController pid;
  private final ArmRotationEncoder c_armRotationEncoder;
  private final double TOLERANCE = 0.5;

  final double kP = 0.0;
  final double kI = 0.0;
  final double kD = 0.0;

  public RotateArm(Intake intake, double intakeAngle, ArmRotationEncoder armRotationEncoder) {
    c_intake = intake;
    c_armRotationEncoder = armRotationEncoder;
    c_angle = intakeAngle;
    pid = new PIDController(kP, kI, kD);
    addRequirements(c_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    c_intake.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calculate(c_armRotationEncoder.getShooterEncoderPosition(), c_angle);
    c_intake.rotateArm(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_intake.rotateArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(c_armRotationEncoder.getShooterEncoderPosition() - c_angle) < TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }
}
