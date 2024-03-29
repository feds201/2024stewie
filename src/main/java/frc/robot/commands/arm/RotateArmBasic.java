// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class RotateArmBasic extends Command {
  /** Creates a new RotateArmBasic. */
  DoubleSupplier c_speed;
  Arm c_arm;

  public RotateArmBasic(Arm arm, DoubleSupplier speed) {
    c_arm = arm;
    c_speed = speed;

    addRequirements(c_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c_arm.setOutput(c_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_arm.setOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
