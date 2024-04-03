// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class RotateArmBasic extends Command {
  DoubleSupplier c_speed;
  Arm c_arm;

  public RotateArmBasic(Arm arm, DoubleSupplier speed) {
    c_arm = arm;
    c_speed = speed;

    addRequirements(c_arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    c_arm.setOutput(c_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    c_arm.setOutput(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
