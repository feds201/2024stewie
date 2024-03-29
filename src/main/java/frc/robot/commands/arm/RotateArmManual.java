// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.leds.Leds;

public class RotateArmManual extends Command {
  /** Creates a new RotateArmManual. */

  private final Arm c_arm;
  private final DoubleSupplier c_power;
  private final Leds c_leds;

  public RotateArmManual(Arm arm, DoubleSupplier power,Leds leds) {
    c_arm = arm;
    c_power = power;
    c_leds = leds;

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
    SmartDashboard.putBoolean("BRUH", ArmConstants.DidJuiliaNotPressButton);
      if ( CheckFOR80() ) {
        if (ArmConstants.DidJuiliaNotPressButton) {
        c_leds.setLedColor(Leds.LedColors.YELLOW);
        c_arm.rotateOrHold(ArmConstants.kHoldThreshold);
        return;
        }
      }

      double powerValue = c_power.getAsDouble();

      // TODO: Copied code from https://github.com/feds201/2023Dora/blob/main/src/main/java/frc/robot/commands/arm2/RotateArm2Manual.java
      // Would this be beneficial to more smoother moving?
      // double power = rotatePowerSupplier.getAsDouble();
      powerValue = MathUtil.applyDeadband(powerValue , 0.1);
      powerValue = Math.copySign(Math.pow(powerValue , 2) , powerValue);
      powerValue /= 1.5;

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


  public boolean CheckFOR80() {
    return c_arm.getArmAngle() > 78;
  }


}


