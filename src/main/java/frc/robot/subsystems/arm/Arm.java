// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.CANConstants;
import frc.robot.constants.DIOConstants;
import frc.robot.subsystems.SubsystemABC;

public class Arm extends SubsystemABC {
  /** Creates a new Arm. */
  private final TalonFX armRotation; // FIXME: Set encoder soft limits
  private final DutyCycleEncoder armRotationEncoder;

  private final PIDController pid = ArmConstants.ArmPIDForExternalEncoder.GetArmPID();

  private DoubleEntry armTarget;
  private DoubleEntry armOutput;
  private DoubleEntry armRotationEncoderValue;
  private DoubleEntry armRotationEncoderAngle;
  private DoubleEntry armInternalEncoderValue;
  private DoubleEntry armInternalEncoderAngle;
  private BooleanEntry failure;

  public Arm() {
    super();
    
    armRotation = new TalonFX(CANConstants.Arm.kArm);
    armRotation.getConfigurator().apply(ArmConstants.GetArmMotorConfiguration());
    armRotationEncoder = new DutyCycleEncoder(DIOConstants.Arm.kArmRotateEncoder);
    
    setupNetworkTables("arm");
    
    armTarget = ntTable.getDoubleTopic("target").getEntry(0);
    armOutput = ntTable.getDoubleTopic("output").getEntry(0);
    armRotationEncoderValue = ntTable.getDoubleTopic("rotation_value").getEntry(0);
    armRotationEncoderAngle = ntTable.getDoubleTopic("rotation_angle").getEntry(0);
    armInternalEncoderValue = ntTable.getDoubleTopic("rotation_value_internal").getEntry(0);
    armInternalEncoderAngle = ntTable.getDoubleTopic("rotation_angle_internal").getEntry(0);
    failure = ntTable.getBooleanTopic("rotation_angle_internal").getEntry(false);

    armRotationEncoder.setPositionOffset(0.3473);

    setupShuffleboard();
    seedNetworkTables();
  }

  @Override
  public void seedNetworkTables() {
    setOutput(0);
    setTarget(0);
    getOutput();
    getTarget();
  }

  @Override
  public void setupShuffleboard() {
    tab.add("armRotation talon", armRotation);
    tab.add("armRotationEncoder", armRotationEncoder);
    tab.add("pid controller", pid);
  }

  public boolean isArmAtTarget() {
    return pid.atSetpoint();
  }

  public void rotateArmToTarget() {
    SmartDashboard.putNumber("arm pid", pid.calculate(getArmAngle()));
    this.setOutput(pid.calculate(getArmAngle()));
  }

  public void setPIDTarget(double target) {
    this.setTarget(target);
    pid.setSetpoint(target);
  }

  public void stopArmRotation() {
    this.setOutput(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    writePeriodicOutputs();
  }

  @Override
  public void writePeriodicOutputs() {
    readArmAngle();
    readRotationEncoder();
  }

  // GETTERS
  public double getTarget() {
    return armTarget.get();
  }

  public double getOutput() {
    return armOutput.get();
  }

  public double getArmAngle() {
    return armRotationEncoderAngle.get();
  }

  public double getEncoderValue() {
    return armRotationEncoderValue.get();
  }

  public boolean getFailure() {
    return failure.get();
  }

  private DoubleLogEntry armTargetLog = new DoubleLogEntry(log, "/arm/target");
  private DoubleLogEntry armOutputLog = new DoubleLogEntry(log, "/arm/output");
  private DoubleLogEntry armRotationEncoderValueLog = new DoubleLogEntry(log, "/arm/rotationValue");
  private DoubleLogEntry armRotationEncoderAngleLog = new DoubleLogEntry(log, "/arm/rotationAngle");
  private DoubleLogEntry armInternalEncoderValueLog = new DoubleLogEntry(log, "/arm/internalValue");
  private DoubleLogEntry armInternalEncoderAngleLog = new DoubleLogEntry(log, "/arm/internalAngle");
  private BooleanLogEntry failureLog = new BooleanLogEntry(log, "/arm/failure");

  // SETTERS
  public void setOutput(double output) {
    armOutput.set(output);
    armOutputLog.append(armOutput.get());

    VoltageOut voltage = new VoltageOut(output);

    armRotation.setControl(voltage.withOutput(output));
  }

  public void setTarget(double target) {
    armTarget.set(target);
    armTargetLog.append(target);
  }

  public void setFailure(boolean failureValue) {
    failure.set(failureValue);
    failureLog.append(failureValue);
  }

  public void readArmAngle() {
    armRotationEncoderAngle.set(armRotationEncoder.get() * 360);
    armRotationEncoderAngleLog.append(armRotationEncoderAngle.get());
  }

  public void readRotationEncoder() {
    armRotationEncoderValue.set(armRotationEncoder.get());
    armRotationEncoderValueLog.append(armRotationEncoderValue.get());
  }

  public void readArmAngleInternal() {
    armInternalEncoderAngle.set(armRotation.getPosition().getValueAsDouble() / ArmConstants.kArmGearReduction);
    armInternalEncoderAngleLog.append(armInternalEncoderAngle.get());
  }

  public void readInternalEncoder() {
    armInternalEncoderValue.set(armRotation.getPosition().getValueAsDouble());
    armInternalEncoderValueLog.append(armInternalEncoderValue.get());
  }
}
