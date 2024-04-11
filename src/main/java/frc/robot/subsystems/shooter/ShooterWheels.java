// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
  private final DoubleEntry shootVelocityMotionMagicTop;
  
  private final DoubleEntry shootVelocityMotionMagicBottom;

  private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);

  public ShooterWheels() {
    super();
    shooterTopMain = new TalonFX(CANConstants.Shooter.kShooterTop);
    shooterBottomFollower = new TalonFX(CANConstants.Shooter.kShooterBottom);
    shooterTopMain.getConfigurator().apply(ShooterConstants.GetWheelsConfiguration());
    shooterBottomFollower.getConfigurator().apply(ShooterConstants.GetWheelsConfiguration());
//    shooterBottomFollower.setControl(new StrictFollower(shooterTopMain.getDeviceID()));

    SignalLogger.start();
    SignalLogger.setPath("/media/sda1/ctre-logs/");

    setupNetworkTables("shooter");
    shootVelocity = ntTable.getDoubleTopic("shoot_velocity").getEntry(0);
    
    shootVoltage = ntTable.getDoubleTopic("shoot_voltage").getEntry(0);
    shootVelocityMotionMagicTop     = ntTable.getDoubleTopic("shoot_velocity_motion_magic_top").getEntry(0);
    shootVelocityMotionMagicBottom  = ntTable.getDoubleTopic("shoot_velocity_motion_magic_bottom").getEntry(0);
 
    setupShuffleboard();
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
  public void seedNetworkTables() {
    setShootVelocity(0);
    setShootVoltage(0);
    setShootVelocityMotionMagic(0, 0);
    getShootVelocity();
    getShootVoltage();
    getShootVelocityMotionMagicTop();
    getShootVelocityMotionMagicBottom();
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

  public double getShootVelocityMotionMagicTop() {
    return shootVelocityMotionMagicTop.get();
  }
  
  public double getShootVelocityMotionMagicBottom() { return shootVelocityMotionMagicBottom.get(); }

  private final DoubleLogEntry shootVelocityLog = new DoubleLogEntry(log, "/shooter/velocity");
  private final DoubleLogEntry shootVoltageLog = new DoubleLogEntry(log, "/shooter/voltage");
  private final DoubleLogEntry shootVelocityMotionMagicTopLog = new DoubleLogEntry(log, "/shooter/velocityMotionMagicTop");
  private final DoubleLogEntry shootVelocityMotionMagicBottomLog = new DoubleLogEntry(log, "/shooter/velocityMotionMagicBottom");

  // SETTERS
  public void setShootVelocity(double velocity) {
    shootVelocity.set(velocity);
    shootVelocityLog.append(velocity);

    VelocityVoltage velocityOut = new VelocityVoltage(0);
    velocityOut.Slot = 0;
    shooterTopMain.setControl(velocityOut.withVelocity(velocity));
    shooterBottomFollower.setControl(velocityOut.withVelocity(velocity));
  }

  public void setShootVelocityMotionMagic(double velocityTop, double velocityBottom) {
    shootVelocityMotionMagicTop.set(velocityTop);
    shootVelocityMotionMagicTopLog.append(velocityTop);
    
    shootVelocityMotionMagicBottom.set(velocityBottom);
    shootVelocityMotionMagicBottomLog.append(velocityBottom);

    motionMagic.Slot = 0;
    shooterTopMain.setControl(motionMagic.withVelocity(velocityTop));
    shooterBottomFollower.setControl(motionMagic.withVelocity(velocityBottom));
  }

  public void setShootVoltage(double voltage) {
    shootVoltage.set(voltage);
    shootVoltageLog.append(voltage);

    DutyCycleOut voltageOut = new DutyCycleOut(0);
    shooterTopMain.setControl(voltageOut.withOutput(voltage));
    shooterBottomFollower.setControl(voltageOut.withOutput(voltage));
  }
}
