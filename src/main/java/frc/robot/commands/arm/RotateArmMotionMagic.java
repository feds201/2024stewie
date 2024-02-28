// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class RotateArmMotionMagic extends Command {
  /** Creates a new RotateArmMotionMagic. */
  private final Arm c_arm;
  private final double c_target;

  public RotateArmMotionMagic(Arm arm, double target) {
    c_arm = arm;
    c_target = target;

    addRequirements(c_arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // c_arm.rotateArmToTargetMotionMagic(c_target);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
