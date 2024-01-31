// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX intakeWheel;
  private final TalonFX intakeRotation;
  private final TalonFX armRotation;
  private final DutyCycleEncoder intakeRotationEncoder;
  private final DutyCycleEncoder armRotationEncoder;

  /** Creates a new Intake. */
  public Intake() {
    intakeWheel = new TalonFX(0);
    intakeRotation = new TalonFX(0);
    armRotation = new TalonFX(0);
    intakeRotationEncoder = new DutyCycleEncoder(0);
    armRotationEncoder = new DutyCycleEncoder(0);

  }

  public void rotateIntakeWheels(double c_intakeVoltage) {
    DutyCycleOut power = new DutyCycleOut(0);
    intakeWheel.setControl(power.withOutput(c_intakeVoltage));
  }

  public void rotateArm(double c_angle) {
    PositionDutyCycle power = new PositionDutyCycle(0);
    intakeWheel.setControl(power.withPosition(c_angle));
  }

  public double getIntakeRotationEncoderPosition() {
    return intakeRotationEncoder.getAbsolutePosition();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
