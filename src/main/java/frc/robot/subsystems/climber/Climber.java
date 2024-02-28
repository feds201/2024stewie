// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import frc.robot.constants.CANConstants;
import frc.robot.subsystems.SubsystemABC;

public class Climber extends SubsystemABC {
  private final CANSparkMax climberMain;
  private final CANSparkMax climberFollower;

  private DoubleEntry climberVoltage;

  /** Creates a new Climber. */
  public Climber() {
    super();
    climberMain = new CANSparkMax(CANConstants.Climber.kClimberRightMain, MotorType.kBrushless);
    climberFollower = new CANSparkMax(CANConstants.Climber.kClimberLeftFollower, MotorType.kBrushless);

    climberFollower.follow(climberMain);
    
    setupNetworkTables("climber");
    climberVoltage = ntTable.getDoubleTopic("climber_voltage").getEntry(0);

    setupShuffleboard();
    seedNetworkTables();
  }
  
  @Override
  public void setupShuffleboard() {
    // tab.add("climber main", climberMain);
    // tab.add("climber follower", climberFollower);
  }

  @Override
  public void seedNetworkTables() {
    setClimberVoltage(0);
    getClimberVoltage();
  }

  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  @Override
  public void writePeriodicOutputs() {
    // write encoder values here!
  }
  
  
  // GETTERS  
  public double getClimberVoltage() {
    return climberVoltage.get();
  }


  private DoubleLogEntry climberVoltageLog = new DoubleLogEntry(log, "/climber/voltage");

  // SETTERS
  public void setClimberVoltage(double voltage) {
    climberVoltage.set(voltage);
    climberVoltageLog.append(climberVoltage.get());

    climberMain.set(climberVoltage.get());
  }
}
