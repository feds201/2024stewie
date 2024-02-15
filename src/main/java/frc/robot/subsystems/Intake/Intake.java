// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.CANConstants;
import frc.robot.constants.DIOConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.SubsystemABC;

public class Intake extends SubsystemABC {
  private final CANSparkMax intakeWheel;
  private final CANSparkMax wristRotation;

  private final DutyCycleEncoder wristRotationEncoder;

  private final PIDController pid = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

  private DoubleEntry intakeVoltage;
  private DoubleEntry wristVoltage;
  private DoubleEntry rotationEncoderValue;
  private DoubleEntry rotationAngle;
  private DoubleEntry rotationTarget;

  /** Creates a new Intake. */
  public Intake() {
    super();

    intakeWheel = new CANSparkMax(CANConstants.Intake.kIntakeWheels, MotorType.kBrushless);
    wristRotation = new CANSparkMax(CANConstants.Intake.kIntakeWrist, MotorType.kBrushless);
    wristRotationEncoder = new DutyCycleEncoder(DIOConstants.Intake.kIntakeRotateEncoder);

    pid.setIZone(IntakeConstants.kIZone);
    pid.setTolerance(IntakeConstants.kRotationTolerance);

    setupNetworkTables("intake");

    intakeVoltage = ntTable.getDoubleTopic("wheels_voltage").getEntry(0);
    wristVoltage = ntTable.getDoubleTopic("wrist_voltage").getEntry(0);
    rotationEncoderValue = ntTable.getDoubleTopic("rotation_value").getEntry(0);
    rotationAngle = ntTable.getDoubleTopic("rotation_angle").getEntry(0);
    rotationTarget = ntTable.getDoubleTopic("rotation_target").getEntry(0);

    setupShuffleboard();
    setupTestCommands();
    seedNetworkTables();
  }

  @Override
  public void setupShuffleboard() {
    // tab.add("intake wheel", intakeWheel);
    // tab.add("wrist encoder", wristRotationEncoder);
    tab.add("PID Controller", pid);
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
  public void setupTestCommands() {

  }

  @Override
  public void seedNetworkTables() {
    setIntakeWheels(0);
    setWristVoltage(0);
    getIntakeWheels();
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
  public double getIntakeWheels() {
    return intakeVoltage.get();
  }

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

  private DoubleLogEntry intakeVoltageLog = new DoubleLogEntry(log, "/intake/target");
  private DoubleLogEntry wristVoltageLog = new DoubleLogEntry(log, "/intake/output");
  private DoubleLogEntry rotationEncoderValueLog = new DoubleLogEntry(log, "/intake/rotation_value");
  private DoubleLogEntry rotationAngleLog = new DoubleLogEntry(log, "/intake/rotation_angle");
  private DoubleLogEntry rotationTargetLog = new DoubleLogEntry(log, "/intake/rotation_target");

  // SETTERS
  public void setWristVoltage(double voltage) {
    wristVoltage.set(voltage);
    wristVoltageLog.append(voltage);

    wristRotation.setVoltage(voltage);
  }

  public void setIntakeWheels(double voltage) {
    intakeVoltage.set(voltage);
    intakeVoltageLog.append(voltage);

    intakeWheel.setVoltage(voltage);
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
