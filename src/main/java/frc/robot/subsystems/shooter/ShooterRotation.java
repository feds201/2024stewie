// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CANConstants;
import frc.robot.constants.DIOConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.SubsystemABC;

public class ShooterRotation extends SubsystemABC {
  // Motors
  private final TalonFX shooterRotate; // Kraken

  // Encoder
  private final DutyCycleEncoder shooterRotateEncoder; // Through Bore Encoder
  private final DoubleSupplier currentArmRotationSupplier;

  private final PIDController rotatePID = ShooterConstants.RotationPIDForExternalEncoder.GetRotationPID();

  private final DoubleEntry rotateVoltage;
  private final DoubleEntry rotateTarget;
  private final DoubleEntry encoderValue;
  private final DoubleEntry encoderAngleWithoutOffset;
  private final DoubleEntry encoderAngle;
  private final BooleanEntry failure;

  public ShooterRotation(DoubleSupplier currentArmRotationSupplier) {
    super();
    shooterRotate = new TalonFX(CANConstants.Shooter.kShooterPivot);
    shooterRotateEncoder = new DutyCycleEncoder(DIOConstants.Shooter.kShooterRotateEncoder);
    shooterRotate.getConfigurator().apply(ShooterConstants.GetRotationConfiguration());
    this.currentArmRotationSupplier = currentArmRotationSupplier;

    SignalLogger.start();
    SignalLogger.setPath("/media/sda1/ctre-logs/");

    setupNetworkTables("shooter");
    rotateVoltage = ntTable.getDoubleTopic("rotate_angle").getEntry(0);
    rotateTarget = ntTable.getDoubleTopic("rotate_target").getEntry(0);
    encoderValue = ntTable.getDoubleTopic("encoder_value").getEntry(0);
    encoderAngleWithoutOffset = ntTable.getDoubleTopic("encoder_angle_no_offset").getEntry(0);
    encoderAngle = ntTable.getDoubleTopic("encoder_angle").getEntry(0);
    failure = ntTable.getBooleanTopic("failure").getEntry(false);

    shooterRotateEncoder.setPositionOffset(0.9170); // REMEMBER TO RESET AT HOME

    setupShuffleboard();
    seedNetworkTables();
  }

  @Override
  public void setupShuffleboard() {
    tab.add("shooter rotate encoder", shooterRotateEncoder);
    tab.add("shoote rotate motor", shooterRotate);
    tab.add("rotation pid controller", rotatePID);
  }

  @Override
  public void writePeriodicOutputs() {
    readEncoderAngle();
    readEncoderAngleWithoutOffset();
    readEncoderValue();
  }

  @Override
  public void seedNetworkTables() {
    setRotateTarget(0.001);
    setRotateVoltage(0);
    getRotateAngle();
    getRotateTarget();
  }

  public void setPIDTarget(double target) {
    setRotateTarget(target);

    rotatePID.setSetpoint(target);
  }

  public boolean pidAtSetpoint() {
    return rotatePID.atSetpoint();
  }

  public void rotateShooterPID() {
    double currentAngle = getEncoderAngle();
    if (currentAngle < -60 || currentAngle > 10) {
      setFailure(true);
    } else {
      double output = rotatePID.calculate(currentAngle);
      SmartDashboard.putNumber("current target", rotatePID.getSetpoint());
      SmartDashboard.putNumber("current angle", getEncoderAngle());
      SmartDashboard.putNumber("current output", output);
      SmartDashboard.putNumber("current kP", rotatePID.getP());
      SmartDashboard.putNumber("current kI", rotatePID.getI());
      SmartDashboard.putNumber("current kD", rotatePID.getD());

      setRotateVoltage(output); // positive direction is towards intake, when it should be away from intake
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    writePeriodicOutputs();
  }

  // GETTERS
  public double getRotateAngle() {
    return rotateVoltage.get();
  }

  public double getRotateTarget() {
    return rotateTarget.get();
  }

  public double getEncoderValue() {
    return encoderValue.get();
  }

  public double getEncoderAngleWithoutOffset() {
    return encoderAngleWithoutOffset.get();
  }

  public double getEncoderAngle() {
    return encoderAngle.get();
  }

  public boolean getFailure() {
    return failure.get();
  }

  private final DoubleLogEntry rotateVoltageLog = new DoubleLogEntry(log, "/shooter/angle");
  private final DoubleLogEntry rotateTargetLog = new DoubleLogEntry(log, "/shooter/target");
  private final DoubleLogEntry encoderValueLog = new DoubleLogEntry(log, "/shooter/encoderValue");
  private final DoubleLogEntry encoderAngleWithoutOffsetLog = new DoubleLogEntry(log,
      "/shooter/encoderAngleNoOffset");
  private final DoubleLogEntry encoderAngleLog = new DoubleLogEntry(log, "/shooter/encoderAngle");
  private final BooleanLogEntry failureLog = new BooleanLogEntry(log, "/shooter/failure");

  public void setRotateVoltage(double voltage) {
    rotateVoltage.set(voltage);
    rotateVoltageLog.append(voltage);

    DutyCycleOut angleVolts = new DutyCycleOut(0);
    shooterRotate.setControl(angleVolts.withOutput(voltage));
  }

  public void setRotateTarget(double target) {
    rotateTarget.set(target);
    rotateTargetLog.append(target);
  }

  public void setFailure(boolean failureValue) {
    failure.set(failureValue);
    failureLog.append(failureValue);
  }

  public void readEncoderValue() {
    double encoder = shooterRotateEncoder.get();
    if(encoder < -340/360) {
      encoder += 1;
    } else if (encoder > 10/360) {
      encoder -= 1;
    }
    encoderValue.set(encoder);
    encoderValueLog.append(encoder);
  }

  public void readEncoderAngleWithoutOffset() {
    double angle = encoderValue.get() * 360;
    if(angle < -340) {
      angle += 360;
    } else if (angle > 10) {
      angle -= 360;
    }
    encoderAngleWithoutOffset.set(angle);
    encoderAngleWithoutOffsetLog.append(angle);
  }

  public void readEncoderAngle() {
    double angle = shooterRotateEncoder.get() * 360 - currentArmRotationSupplier.getAsDouble();
    if(angle < -340) {
      angle += 360;
    } else if (angle > 10) {
      angle -= 360;
    }
    encoderAngle.set(angle);
    encoderAngleLog.append(angle);
  }

}
