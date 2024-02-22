// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.distance_sensor;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemABC;
import frc.robot.subsystems.distance_sensor.SensorManager;

public class DistanceSensor extends SubsystemABC {
  private Rev2mDistanceSensor sensor;

  private DoubleEntry sensorRange;
  private StringEntry sensorName;
  private DoubleLogEntry sensorRangeLog;
  private StringLogEntry sensorNameLog;

  public DistanceSensor(Rev2mDistanceSensor.Port sensorType) {
    super();

    sensorRange = ntTable.getDoubleTopic("sensor_range").getEntry(0);
    sensorName = ntTable.getStringTopic("sensor_name").getEntry(sensorType.toString());

    sensorRangeLog = new DoubleLogEntry(log, "/distanceSensor" + sensorType.toString() + "/sensorRange");
    sensorNameLog = new StringLogEntry(log, "/distanceSensor" + sensorType.toString() + "/sensorName");
    
    setSensorName(sensorType.toString());

    sensor = new Rev2mDistanceSensor(sensorType);
    sensor.setAutomaticMode(true);
    
    seedNetworkTables();
  }

  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  @Override
  public void seedNetworkTables() {
    // Seed Network Tables is only really necessary for setters that the Programmer
    // sets,
    // not a sensor that "read"s independently of the programmer
    
  }

  @Override
  public void writePeriodicOutputs() {
    readSensorRange();
    getSensorName(); // TODO: is this really necessary to get repeatedly in periodic??
  }

  @Override
  public void setupTestCommands() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method
    // 'setupTestCommands'");
  }

  @Override
  public void setupShuffleboard() {
    // tab.add("sensor", sensor); // this won't work since Rev2MDistanceSensor does
    // not implement Sendable
  }

  public double getSensorRange() {
    return sensorRange.get();
  }

  public String getSensorName() {
    return sensorName.get();
  }

  // SETTERS

  public void readSensorRange() {
    sensorRange.set(sensor.getRange());
    sensorRangeLog.append(sensorRange.get());
  }

  private void setSensorName(String name) {
    sensorName.set(name);
    sensorNameLog.append(sensorName.get());
  }
}
