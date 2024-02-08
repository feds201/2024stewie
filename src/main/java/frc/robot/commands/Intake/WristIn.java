// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

import java.lang.Math;

public class WristIn extends Command {
  /** Creates a new wristIn. */
  private final Intake c_intake;
  private final PIDController pid;
  private final double c_target;
  private final double TOLERANCE = 0.5;

  final double kP = 0.0;
  final double kI = 0.0;
  final double kD = 0.0;

  public WristIn(Intake intake, double target) {
    c_intake = intake;
    c_target = target;
    pid = new PIDController(kP, kI, kD);

    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calculate(c_intake.getWristRotationEncoderPosition(), c_target);
    c_intake.rotateWrist(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_intake.rotateWrist(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(c_intake.getWristRotationEncoderPosition() - c_target) < TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }
}