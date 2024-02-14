// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.CANConstants;
import frc.robot.constants.DIOConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeWheel;
  private final CANSparkMax wristRotation;

  public final DutyCycleEncoder wristRotationEncoder;

  /** Creates a new Intake. */
  public Intake() {
    intakeWheel = new CANSparkMax(CANConstants.Intake.kIntakeWheels, MotorType.kBrushless);
    wristRotation = new CANSparkMax(CANConstants.Intake.kIntakeWrist, MotorType.kBrushless);
    wristRotationEncoder = new DutyCycleEncoder(DIOConstants.Intake.kIntakeRotateEncoder);
  }

  public void rotateIntakeWheels(double c_intakeVoltage) {
    intakeWheel.setVoltage(c_intakeVoltage);
  }

  public void rotateWrist(double c_wristVoltage) {
    wristRotation.setVoltage(c_wristVoltage);
  }

  public double getWristRotationEncoderPosition() {
    return wristRotationEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
