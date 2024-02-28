// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CANConstants;
import frc.robot.constants.DIOConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.SubsystemABC;

public class Wrist extends SubsystemABC {
  private final CANSparkMax wristRotation;
  private final DutyCycleEncoder wristRotationEncoder;

  private final PIDController pid = IntakeConstants.WristPID.GetWristPID();

  private DoubleEntry wristVoltage;
  private DoubleEntry rotationEncoderValue;
  private DoubleEntry rotationAngle;
  private DoubleEntry rotationTarget;

  /** Creates a new Intake. */
  public Wrist() {
    super();

    wristRotation = new CANSparkMax(CANConstants.Intake.kIntakeWrist, MotorType.kBrushless);
    wristRotationEncoder = new DutyCycleEncoder(DIOConstants.Intake.kIntakeRotateEncoder);

    setupNetworkTables("intake");

    wristVoltage = ntTable.getDoubleTopic("wrist_voltage").getEntry(0);
    rotationEncoderValue = ntTable.getDoubleTopic("rotation_value").getEntry(0);
    rotationAngle = ntTable.getDoubleTopic("rotation_angle").getEntry(0);
    rotationTarget = ntTable.getDoubleTopic("rotation_target").getEntry(0);

    SmartDashboard.putNumber("WRIST BEFORE", wristRotationEncoder.get());
    wristRotationEncoder.setPositionOffset(0.7904);
    SmartDashboard.putNumber("WRIST AFTER", wristRotationEncoder.get());

    setupShuffleboard();
    seedNetworkTables();
  }

  @Override
  public void setupShuffleboard() {
    tab.add("PID Controller", pid);
    // tab.add("Wrist Voltage", wristVoltage);
    // tab.add("Rotation Encoder Value", rotationEncoderValue);
    // tab.add("Rotation Angle", rotationAngle);
    // tab.add("Rotation Target", rotationTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    writePeriodicOutputs();
  }

  @Override
  public void writePeriodicOutputs() {
    readWristAngle();
    readIntakeEncoder();
  }

  @Override
  public void seedNetworkTables() {
    setWristVoltage(0);
    getWristVoltage();
  }

  public void rotateWristPID() {
    double output = pid.calculate(getWristAngle());
    setWristVoltage(output);
  }

  public void setPIDTarget(double target) {
    setTarget(target);
    pid.setSetpoint(target);
  }

  public boolean pidAtSetpoint() {
    return pid.atSetpoint();
  }

  // GETTERS
  public double getWristVoltage() {
    return wristVoltage.get();
  }

  public double getWristAngle() {
    return rotationAngle.get();
  }

  public double getEncoderValue() {
    return rotationEncoderValue.get();
  }

  public double getTarget() {
    return rotationTarget.get();
  }

  private DoubleLogEntry wristVoltageLog = new DoubleLogEntry(log, "/intake/output");
  private DoubleLogEntry rotationEncoderValueLog = new DoubleLogEntry(log, "/intake/rotationValue");
  private DoubleLogEntry rotationAngleLog = new DoubleLogEntry(log, "/intake/rotationAngle");
  private DoubleLogEntry rotationTargetLog = new DoubleLogEntry(log, "/intake/rotationTarget");

  // SETTERS
  public void setWristVoltage(double voltage) {
    wristVoltage.set(voltage);
    wristVoltageLog.append(voltage);

    wristRotation.set(voltage);
  }

  public void readWristAngle() {
    rotationAngle.set(wristRotationEncoder.get() * 360);
    rotationAngleLog.append(rotationAngle.get());
  }

  public void readIntakeEncoder() {
    rotationEncoderValue.set(wristRotationEncoder.get());
    rotationEncoderValueLog.append(rotationEncoderValue.get());
  }

  public void setTarget(double target) {
    rotationTarget.set(target);
    rotationTargetLog.append(target);
  }
}
