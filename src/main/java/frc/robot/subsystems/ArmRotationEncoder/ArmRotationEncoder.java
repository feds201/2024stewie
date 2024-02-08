// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmRotationEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotationEncoder extends SubsystemBase {
    private final DutyCycleEncoder shooterRotateEncoder;
   
  /** Creates a new ArmRotationEncoder. */
  public ArmRotationEncoder() {
    shooterRotateEncoder = new DutyCycleEncoder(9);
      shooterRotateEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("current encoder position", getShooterEncoderPosition());
  }
  public double getShooterEncoderPosition() {
    return shooterRotateEncoder.get() * 360;
  }
  public void resetEncoder(){
    shooterRotateEncoder.reset();
  }
}
