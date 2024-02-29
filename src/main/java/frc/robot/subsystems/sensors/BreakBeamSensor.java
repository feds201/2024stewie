// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.DIOConstants.*;
import frc.robot.subsystems.SubsystemABC;

public class BreakBeamSensor extends SubsystemABC {
  /** Creates a new BreakBeamSensor. */
  // private final DigitalInput transmitter;
  private final DigitalInput receiver;
  private final BooleanEntry beamBroken;

  public BreakBeamSensor() {
    setupNetworkTables("irsensor");

    // transmitter = new DigitalInput(SensorConstants.breakBeamTransmitterPort);
    receiver = new DigitalInput(SensorConstants.breakBeamReceiverPort);

    beamBroken = ntTable.getBooleanTopic("shooter_loaded").getEntry(true);

    setupShuffleboard();
    seedNetworkTables();
  }

  @Override
  public void setupShuffleboard() {
    tab.add("BreakBeam", receiver);
  }

  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  @Override
  public void seedNetworkTables() {
  }

  @Override
  public void writePeriodicOutputs() {
    readBeamBroken();
  }

  public void readBeamBroken() {
    beamBroken.set(!receiver.get());
  }

  public boolean getBeamBroken() {
    return beamBroken.get();
  }

}
