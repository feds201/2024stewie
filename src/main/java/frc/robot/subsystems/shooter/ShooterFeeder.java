// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Servo;


import frc.robot.subsystems.SubsystemABC;


public class ShooterFeeder extends SubsystemABC {
  /** Creates a new ShooterFeeder. */
  private final Servo feeder;
  private DoubleEntry currentServoSpeed;
  public ShooterFeeder(int feederPort) {
    super();
    feeder = new Servo(feederPort);
    setupNetworkTables("shooter");
    currentServoSpeed = ntTable.getDoubleTopic("current_servo_speed").getEntry(0);

    setupShuffleboard();
    setupTestCommands();
    seedNetworkTables();
  }

  @Override
  public void periodic() {
   writePeriodicOutputs();
  }

  @Override
  public void seedNetworkTables() {
    setCurrentServoSpeed(0.5);
    getCurrentServoSpeed();
  }

  @Override
  public void writePeriodicOutputs() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void setupTestCommands() {
    // TODO Auto-generated method stub
   
  }

  @Override
  public void setupShuffleboard() {
    // TODO Auto-generated method stub
   
  }
  //GETTERS
  public double getCurrentServoSpeed() {
    return currentServoSpeed.get();
  }
  private DoubleLogEntry currentServoSpeedLog = new DoubleLogEntry(log, "/servo/speed" );

  //SETTERS 
  public void setCurrentServoSpeed(double speed){
    currentServoSpeed.set(speed);
    currentServoSpeedLog.append(currentServoSpeed.get());

    //feeder.setSpeed(speed);
    feeder.set(speed);
  }
}
