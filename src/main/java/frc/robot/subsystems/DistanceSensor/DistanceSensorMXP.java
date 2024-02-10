// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DistanceSensor;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensorMXP extends SubsystemBase {
  private Rev2mDistanceSensor sensorMXP;

  public DistanceSensorMXP() {
    sensorMXP = new Rev2mDistanceSensor(Port.kMXP);
    sensorMXP.setAutomaticMode(true);
    setupDebug();
  }
  
  @Override
  public void periodic() {

    if (sensorMXP.isRangeValid()) {
      SmartDashboard.putNumber("Range MXP", sensorMXP.getRange());
      SmartDashboard.putNumber("Timestamp MXP", sensorMXP.getTimestamp());

      SmartDashboard.putString("Distance Units MXP", sensorMXP.getDistanceUnits().toString());
      SmartDashboard.putNumber("Some other range idk MXP", sensorMXP.GetRange());

      SmartDashboard.putBoolean("In Range MXP", sensorMXP.getRange() < 4);
    }
    // if(distOnboard.isRangeValid()) {
    // SmartDashboard.putNumber("Range Onboard", distOnboard.getRange());
    // SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp());
    // }

    // if(distMXP.isRangeValid()) {
    // SmartDashboard.putNumber("Range MXP", distMXP.getRange());
    // SmartDashboard.putNumber("Timestamp MXP", distMXP.getTimestamp());
    // }
  }

  private void setupDebug() {

  }

}
