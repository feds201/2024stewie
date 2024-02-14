// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX armRotation;
  private final DutyCycleEncoder armRotationEncoder;

  private final double kP = 0.0;
  private final double kI = 0.0;
  private final double kD = 0.0;

  private final PIDController pid = new PIDController(kP, kI, kD);

  public Arm() {
    armRotation = new TalonFX(CANConstants.Arm.kArm);
    armRotationEncoder = new DutyCycleEncoder(0);

    pid.setTolerance(5, 10);
    pid.setIZone(Double.POSITIVE_INFINITY);
  }

  public void rotateArm(double target) {
    pid.calculate(armRotationEncoder.get(), target);
    armRotation.set(pid.calculate(armRotationEncoder.get(), target));
    pid.atSetpoint();
  }

  public void resetEncoder() {
    armRotationEncoder.reset();
  }

  public double getEncoderValue() {
    return armRotationEncoder.get();
  }

  public double getArmAngle() {
    return this.getEncoderValue() * 360;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
