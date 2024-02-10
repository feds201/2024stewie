// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DistanceSensor;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensor extends SubsystemBase {
  private final Rev2mDistanceSensor sensor;

  public DistanceSensor() {
    sensor = new Rev2mDistanceSensor(Port.kOnboard);
    sensor.setAutomaticMode(true);
    setupDebug();
  }

  @Override
  public void periodic() {
    if (sensor.isRangeValid()) {
      SmartDashboard.putNumber("Range", sensor.getRange());
      SmartDashboard.putNumber("Timestamp", sensor.getTimestamp());

      SmartDashboard.putString("Distance Units", sensor.getDistanceUnits().toString());
      SmartDashboard.putNumber("Some other range idk", sensor.GetRange());

      SmartDashboard.putBoolean("In Range", sensor.getRange() < 4);
    }
  }

  private void setupDebug() {
    
  }
}