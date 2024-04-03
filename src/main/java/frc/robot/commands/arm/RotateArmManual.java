// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.leds.Leds;

public class RotateArmManual extends Command {
  private final Arm c_arm;
  private final DoubleSupplier c_power;


  public RotateArmManual(Arm arm, DoubleSupplier power) {
    c_arm = arm;
    c_power = power;

    addRequirements(c_arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double powerValue = c_power.getAsDouble();

    // Copied code from https://github.com/feds201/2023Dora/blob/main/src/main/java/frc/robot/commands/arm2/RotateArm2Manual.java
    powerValue = MathUtil.applyDeadband(powerValue, 0.1);
    powerValue = Math.copySign(Math.pow(powerValue, 2), powerValue);
    powerValue /= 1.5;

    c_arm.rotateOrHold(powerValue * ArmConstants.kArmSpeedScaler);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
