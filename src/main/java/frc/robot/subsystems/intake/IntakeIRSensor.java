// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.DIOConstants.*;
import frc.robot.subsystems.SubsystemABC;

public class IntakeIRSensor extends SubsystemABC {
  /** Creates a new BreakBeamSensor. */
  // private final DigitalInput transmitter;
  private final DigitalInput receiverIntake;
  private final BooleanEntry beamBrokenIntake;

  public IntakeIRSensor() {
    setupNetworkTables("irsensor_intake");

    // transmitter = new DigitalInput(SensorConstants.breakBeamTransmitterPort);
    receiverIntake = new DigitalInput(SensorConstants.intakeBreakBeamReceiverPort);

    beamBrokenIntake = ntTable.getBooleanTopic("intake_loaded").getEntry(true);

    setupShuffleboard();
    seedNetworkTables();
  }

  @Override
  public void setupShuffleboard() {
    tab.add("BreakBeam", receiverIntake);
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
    beamBrokenIntake.set(!receiverIntake.get());
  }

  public boolean getBeamBroken() {
    return beamBrokenIntake.get();
  }

}
