// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.arm.Arm;

public class RotateArmManual extends Command {
  /** Creates a new RotateArmManual. */

  private final Arm c_arm;
  private final DoubleSupplier c_power;

  public RotateArmManual(Arm arm, DoubleSupplier power) {
    c_arm = arm;
    c_power = power;

    addRequirements(c_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double powerValue = c_power.getAsDouble();

    // TODO: Copied code from https://github.com/feds201/2023Dora/blob/main/src/main/java/frc/robot/commands/arm2/RotateArm2Manual.java
    // Would this be beneficial to more smoother moving?
    // double power = rotatePowerSupplier.getAsDouble();
    powerValue = MathUtil.applyDeadband(powerValue, OIConstants.kDriverDeadzone);
    powerValue = Math.copySign(Math.pow(powerValue, 2), powerValue);
    powerValue /= 2;

    c_arm.rotateOrHold(powerValue * ArmConstants.kArmSpeedScaler);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
