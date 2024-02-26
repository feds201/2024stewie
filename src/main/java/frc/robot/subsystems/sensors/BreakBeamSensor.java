// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DIOConstants.*;
import frc.robot.subsystems.SubsystemABC;

public class BreakBeamSensor extends SubsystemABC {
  /** Creates a new BreakBeamSensor. */
  //private final DigitalInput transmitter;
  private final DigitalInput receiver;
  private final BooleanEntry beamBroken;

  public BreakBeamSensor() {
    setupNetworkTables("irsensor");

    //transmitter = new DigitalInput(SensorConstants.breakBeamTransmitterPort);
    receiver = new DigitalInput(SensorConstants.breakBeamReceiverPort);

    beamBroken = ntTable.getBooleanTopic("shooter_loaded").getEntry(true);
  }

  public boolean shooterLoaded() {
    return !receiver.get();
  }
  
  @Override
  public void periodic() {
    beamBroken.set(shooterLoaded());
  }

  @Override
  public void seedNetworkTables() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'seedNetworkTables'");
  }

  @Override
  public void writePeriodicOutputs() {
    // receiver.get();
    SmartDashboard.putBoolean("Beam Broken", receiver.get());
  }

  @Override
  public void setupTestCommands() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setupTestCommands'");
  }

  @Override
  public void setupShuffleboard() {
    tab.add("BreakBeam", receiver);
  }
}
