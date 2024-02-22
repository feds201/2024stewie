// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.SubsystemABC;

public class ShooterWheels extends SubsystemABC {
  // Motors
  private final TalonFX shooterTopMain; // Falcon
  private final TalonFX shooterBottomFollower; // Falcon

  private final DoubleEntry shootVelocity;
  private final DoubleEntry shootVoltage;

  public ShooterWheels() {
    super();
    shooterTopMain = new TalonFX(CANConstants.Shooter.kShooterTop);
    shooterBottomFollower = new TalonFX(CANConstants.Shooter.kShooterBottom);

    SignalLogger.start();
    SignalLogger.setPath("/media/sda1/ctre-logs/");

    shooterTopMain.getConfigurator().apply(ShooterConstants.GetShooterConfiguration());
    shooterBottomFollower.setControl(new StrictFollower(shooterTopMain.getDeviceID()));

    setupNetworkTables("shooterWheels");
    shootVelocity = ntTable.getDoubleTopic("shoot_velocity").getEntry(0);
    shootVoltage = ntTable.getDoubleTopic("shoot_voltage").getEntry(0);
 
    setupShuffleboard();
    setupTestCommands();
    seedNetworkTables();
  }

  @Override
  public void setupShuffleboard() {
    tab.add("shooter top main", shooterTopMain);
    tab.add("shooter botom follower", shooterBottomFollower);
  }

  @Override
  public void writePeriodicOutputs() {
    // no encoder
  }

  @Override
  public void setupTestCommands() {
    
  }

  @Override
  public void seedNetworkTables() {
    setShootVelocity(0);
    setShootVoltage(0);
    getShootVelocity();
    getShootVoltage();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    writePeriodicOutputs();
  }

  // GETTERS
  public double getShootVelocity() {
    return shootVelocity.get();
  }

  public double getShootVoltage() {
    return shootVoltage.get();
  }

  private final DoubleLogEntry shootVelocityLog = new DoubleLogEntry(log, "/shooter/velocity");
  private final DoubleLogEntry shootVoltageLog = new DoubleLogEntry(log, "/shooter/voltage");

  // SETTERS
  public void setShootVelocity(double velocity) {
    shootVelocity.set(velocity);
    shootVelocityLog.append(velocity);

    VelocityVoltage velocityOut = new VelocityVoltage(0);
    velocityOut.Slot = 0;
    shooterTopMain.setControl(velocityOut.withVelocity(velocity));
  }

  public void setShootVoltage(double voltage) {
    shootVoltage.set(voltage);
    shootVoltageLog.append(voltage);

    DutyCycleOut voltageOut = new DutyCycleOut(0);
    shooterTopMain.setControl(voltageOut.withOutput(voltage));
  }
}
