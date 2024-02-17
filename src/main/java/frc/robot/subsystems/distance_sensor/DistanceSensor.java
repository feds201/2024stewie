// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.distance_sensor;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemABC;
import frc.robot.subsystems.distance_sensor.SensorManager;

public class DistanceSensor extends SubsystemABC {
  private Rev2mDistanceSensor sensor;
  private String sensorName;
  private DoubleEntry sensorRange;

  public DistanceSensor(Rev2mDistanceSensor.Port sensorType) {
    super(sensorType.toString());
    sensorName = sensorType.toString();

    System.out.println(sensorName);
    sensor = new Rev2mDistanceSensor(sensorType);
    sensor.setAutomaticMode(true);
    setupDebug();

    seedNetworkTables();
  }

  @Override
  public void periodic() {

    writePeriodicOutputs();

  }

  private void setupDebug() {

  }

  @Override
  public void seedNetworkTables() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'seedNetworkTables'");
  }

  @Override
  public void writePeriodicOutputs() {
    // TODO Auto-generated method stub
    if (sensor.isRangeValid()) {
      SmartDashboard.putNumber("Range"+ sensorName, sensor.getRange());
    }

    // if (sensor.isRangeValid()) {
    // SmartDashboard.putNumber("Range Onboard", distOnboard.getRange());
    // SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp());
    // }

    // if(distMXP.isRangeValid()) {
    // SmartDashboard.putNumber("Range MXP", distMXP.getRange());
    // SmartDashboard.putNumber("Timestamp MXP", distMXP.getTimestamp());
    // }


  }

  @Override
  public void setupTestCommands() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setupTestCommands'");
  }

  @Override
  public void setupShuffleboard() {
    tab.add("DistanceSensor " + sensorName, sensor);

  }

}
