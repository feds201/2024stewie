// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  private final CANSparkMax intakeWheel;
  private final CANSparkMax wristRotation;
  private final CANSparkMax armRotation;
  public final DutyCycleEncoder wristRotationEncoder;
  private final DutyCycleEncoder armRotationEncoder;

  private final double kP = 0.0;
  private final double kI = 0.0;
  private final double kD = 0.0;

  private final PIDController pid;

  /** Creates a new Intake. */
  public Intake() {
    intakeWheel = new CANSparkMax(56, MotorType.kBrushless);
    wristRotation = new CANSparkMax(57, MotorType.kBrushless);
    armRotation = new CANSparkMax(0, MotorType.kBrushless);
    wristRotationEncoder = new DutyCycleEncoder(0);
    armRotationEncoder = new DutyCycleEncoder(0);

    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(5, 10);
    pid.setIZone(Double.POSITIVE_INFINITY);
  }



  public void rotateIntakeWheels(double c_intakeVoltage) {
    intakeWheel.setVoltage(c_intakeVoltage);
  }

  public void rotateArm(double target) {
    pid.calculate(armRotationEncoder.get(), target);
    armRotation.set(pid.calculate(armRotationEncoder.get(), target));
    pid.atSetpoint();
  }

  public void rotateWrist(double c_wristVoltage) {
    wristRotation.setVoltage(c_wristVoltage);
  }

  public void resetEncoder() {
    armRotationEncoder.reset();
  }

  public double getWristRotationEncoderPosition() {
    return wristRotationEncoder.getAbsolutePosition();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
