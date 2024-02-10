// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterEncoder extends SubsystemBase {


    public final DutyCycleEncoder intakeRotationEncoder;
    public final DutyCycleEncoder shooterRotateEncoder;

  /** Creates a new ArmEncoder. */
  public ShooterEncoder() {

    //declare encoders
    intakeRotationEncoder = new DutyCycleEncoder(0);
    shooterRotateEncoder = new DutyCycleEncoder(0);
    
  }


  public double getShooterAngle() {
    // Correct calculation and potential conversion
    double encoderDifference = shooterRotateEncoder.getDistance() - intakeRotationEncoder.getDistance();
    return encoderDifference;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}