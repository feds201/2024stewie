// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import frc.robot.constants.DistanceSensorConstants;
import frc.robot.subsystems.SubsystemABC;

public class DistanceSensor extends SubsystemABC {
  private Rev2mDistanceSensor sensor;

  private DoubleEntry sensorRange;
  private StringEntry sensorName;
  private DoubleLogEntry sensorRangeLog;
  private StringLogEntry sensorNameLog;
  private BooleanEntry isRangeValid;
  private BooleanLogEntry isRangeValidLog;

  public DistanceSensor(Rev2mDistanceSensor.Port sensorType) {
    super();

    sensor = new Rev2mDistanceSensor(sensorType);
    sensor.setAutomaticMode(true);

    setupNetworkTables("distance_sensor");

    sensorRange = ntTable.getDoubleTopic("sensor_range").getEntry(0);
    sensorName = ntTable.getStringTopic("sensor_name").getEntry(sensorType.toString());

    sensorRangeLog = new DoubleLogEntry(log, "/distanceSensor/" + sensorType.toString() + "/sensorRange");
    sensorNameLog = new StringLogEntry(log, "/distanceSensor/" + sensorType.toString() + "/sensorName");

    isRangeValid = ntTable.getBooleanTopic("is_range_valid").getEntry(false);
    isRangeValidLog = new BooleanLogEntry(log, "/distanceSensor/" + sensorType.toString() + "/isRangeValid");

    setupShuffleboard();
    seedNetworkTables();
  }

  public boolean noteInIntake() {
    return getSensorRange() > DistanceSensorConstants.kNoteInIntake;
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
    setSensorName("MXP");
    getSensorName();
  }

  @Override
  public void writePeriodicOutputs() {
    readSensorRange();
    getSensorName(); // TODO: is this really necessary to get repeatedly in periodic??
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

  public boolean getIsRangeValid() {
    return isRangeValid.get();
  }

  // SETTERS

  public void readSensorRange() {
    if(sensor.isRangeValid()) {
      sensorRange.set(sensor.getRange());
      sensorRangeLog.append(sensorRange.get());
      isRangeValid.set(true);
      isRangeValidLog.append(true);
    } else {
      isRangeValid.set(false);
      isRangeValidLog.append(false);
    }
    
  }

  private void setSensorName(String name) {
    sensorName.set(name);
    sensorNameLog.append(sensorName.get());
  }
}
