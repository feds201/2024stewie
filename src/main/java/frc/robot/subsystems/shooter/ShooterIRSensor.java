// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.DIOConstants.*;
import frc.robot.subsystems.SubsystemABC;

public class ShooterIRSensor extends SubsystemABC {
  /** Creates a new BreakBeamSensor. */
  // private final DigitalInput transmitter;
  private final DigitalInput receiverShooter;
  private final BooleanEntry beamBrokenShooter;

  public ShooterIRSensor() {
    setupNetworkTables("irsensor_shooter");

    // transmitter = new DigitalInput(SensorConstants.breakBeamTransmitterPort);
    receiverShooter = new DigitalInput(SensorConstants.shooterBreakBeamReceiverPort);

    beamBrokenShooter = ntTable.getBooleanTopic("shooter_loaded").getEntry(true);

    setupShuffleboard();
    seedNetworkTables();
  }

  @Override
  public void setupShuffleboard() {
    tab.add("BreakBeam", receiverShooter);
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
    beamBrokenShooter.set(!receiverShooter.get());
  }

  public boolean getBeamBroken() {
    return beamBrokenShooter.get();
  }

}
