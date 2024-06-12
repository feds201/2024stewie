// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
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
  private final PIDController pidAmp = IntakeConstants.WristPID.GetWristAmpPID();

  private DoubleEntry wristVoltage;
  private DoubleEntry rotationEncoderValue;
  private DoubleEntry rotationAngle;
  private DoubleEntry rotationTarget;

  private BooleanEntry failure;
  private BooleanEntry towardShooter;

  /** Creates a new intake. */
  public Wrist() {
    super();

    wristRotation = new CANSparkMax(CANConstants.Intake.kIntakeWrist, MotorType.kBrushless);
    wristRotationEncoder = new DutyCycleEncoder(DIOConstants.Intake.kIntakeRotateEncoder);

    setupNetworkTables("intake");

    wristVoltage = ntTable.getDoubleTopic("wrist_voltage").getEntry(0);
    rotationEncoderValue = ntTable.getDoubleTopic("rotation_value").getEntry(0);
    rotationAngle = ntTable.getDoubleTopic("rotation_angle").getEntry(0);
    rotationTarget = ntTable.getDoubleTopic("rotation_target").getEntry(0);
    failure = ntTable.getBooleanTopic("failure").getEntry(false);
    towardShooter = ntTable.getBooleanTopic("toward_shooter").getEntry(false);

    wristRotationEncoder.setPositionOffset(0);

    setupShuffleboard();
    seedNetworkTables();
  }


  public boolean isSafe() {
    return false;
  }

  @Override
  public void setupShuffleboard() {
    
    tab.add("PID Controller", pid);
    tab.add("AMP Pid Controller", pidAmp);
  }

  @Override
  public void periodic() {
    wristRotationEncoder.setPositionOffset(0);
     if(!wristRotationEncoder.isConnected()) {
      wristRotation.setVoltage(0);
    }

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
  public void rotateWristPIDAMP() {
    double output = pidAmp.calculate(getWristAngle());
    setWristVoltage(output);
  }

  public void setPIDTarget(double target) {
    setTarget(target);
    pid.setSetpoint(target);
  }
  public void setPIDTargetAMP(double target) {
    setTargetAMP(target);
    pidAmp.setSetpoint(target);
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

  public boolean getFailure() {
    return failure.get();
  }

  private DoubleLogEntry wristVoltageLog = new DoubleLogEntry(log, "/intake/output");
  private DoubleLogEntry rotationEncoderValueLog = new DoubleLogEntry(log, "/intake/rotationValue");
  private DoubleLogEntry rotationAngleLog = new DoubleLogEntry(log, "/intake/rotationAngle");
  private DoubleLogEntry rotationTargetLog = new DoubleLogEntry(log, "/intake/rotationTarget");
  private BooleanLogEntry failureLog = new BooleanLogEntry(log, "/intake/failure");
  private BooleanLogEntry towardShooterLog = new BooleanLogEntry(log, "/intake/towardShooter");

  // SETTERS
  public void setWristVoltage(double voltage) {
    wristVoltage.set(voltage);
    wristVoltageLog.append(voltage);

    if (voltage > 0) {
      setTowardIntake(false);
    } else {
      setTowardIntake(true);
    }

    wristRotation.set(voltage); // TODO FIGURE OUT HOW TO CAP THIS
  }

  public void readWristAngle() {
    double rawEncoderValue = wristRotationEncoder.get();
    double rotationAngleValue = rawEncoderValue * 360;

    SmartDashboard.putNumber("Wrist Encoder Value", rawEncoderValue);
    SmartDashboard.putNumber("Wrist Angle Raw (enc * 360)", rawEncoderValue * 360);

    SmartDashboard.putNumber("Wrist Abs Position", wristRotationEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Wrist Abs Position W/ Offset",
        wristRotationEncoder.getAbsolutePosition() - wristRotationEncoder.getPositionOffset());
    SmartDashboard.putNumber("Wrist Angle Position", wristRotationEncoder.getAbsolutePosition() * 360);
    SmartDashboard.putNumber("Wrist Angle Position W/ Offset",
        (wristRotationEncoder.getAbsolutePosition() - wristRotationEncoder.getPositionOffset()) * 360);

    if (rawEncoderValue < 0) {
      SmartDashboard.putNumber("Encoder negative", rawEncoderValue * 180 + 360);
    } else {
      SmartDashboard.putNumber("Encoder positive", rawEncoderValue * 180);
    }

    // if(rotationAngleValue > 300) {
    // rotationAngleValue -= 360;
    // } else if (rotationAngleValue < -50) {
    // rotationAngleValue += 360;
    // }

    SmartDashboard.putNumber("Wrist after wrap around check", rotationAngleValue);

    rotationAngle.set(rotationAngleValue);
    rotationAngleLog.append(rotationAngleValue);
  }

  public void readIntakeEncoder() {
    double rotationValue = wristRotationEncoder.get();
    if (rotationValue > 300 / 360) {
      rotationValue -= 1;
    } else if (rotationValue < -50 / 360) {
      rotationValue += 1;
    }
    rotationEncoderValue.set(rotationValue);
    rotationEncoderValueLog.append(rotationValue);
  }

  public void setTarget(double target) {
    rotationTarget.set(target);
    rotationTargetLog.append(target);
  }
  public void setTargetAMP(double target) {
    rotationTarget.set(target);
    rotationTargetLog.append(target);
  }

  public void setFailure(boolean failureValue) {
    failure.set(failureValue);
    failureLog.append(failureValue);
  }

  public void setTowardIntake(boolean state) {
    towardShooter.set(state);
    towardShooterLog.append(state);

  }
}




