// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.SubsystemABC;

public class ShooterServos extends SubsystemABC {
  /** Creates a new ShooterFeeder. */
  private final Servo servoThickSide;
  private final Servo servoThinSide;
  // These designations refer to the wheels on the intake being more on
  // one
  // side (thick) and less on the other (thin)

  private final DoubleEntry thickSideSpeed;
  private final DoubleEntry thinSideSpeed;
  private final StringEntry direction;

  public ShooterServos() {
    super();
    servoThickSide = new Servo(ShooterConstants.kThickWheelServoPort);
    servoThinSide = new Servo(ShooterConstants.kThinWheelServoPort);

    setupNetworkTables("shooter");
    thickSideSpeed = ntTable.getDoubleTopic("thick_side_speed").getEntry(0.5);
    thinSideSpeed = ntTable.getDoubleTopic("thin_side_speed").getEntry(0.5);
    direction = ntTable.getStringTopic("servo_direction").getEntry("NONE");

    setupShuffleboard();
    seedNetworkTables();
  }

  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  @Override
  public void seedNetworkTables() {
    setThickSideSpeed(0.5);
    setThinSideSpeed(0.5);
    getThickSideSpeed();
    getThinSideSpeed();
  }

  @Override
  public void writePeriodicOutputs() {

  }

  @Override
  public void setupShuffleboard() {
    // tab.add("Thick side speed", thickSideSpeed);
    // tab.add("Thin side speed", thinSideSpeed);
    // tab.add("Direction", direction);
  }

  public void ejectNote() {
    setThickSideSpeed(ShooterConstants.kServoThickSideSpeed);
    setThinSideSpeed(ShooterConstants.kServoThinSideSpeed);
    setDirection("EJECT");
  }

  public void stopServos() {
    setThickSideSpeed(0.5);
    setThinSideSpeed(0.5);
    setDirection("NONE");
  }

  // GETTERS
  public double getThickSideSpeed() {
    return thickSideSpeed.get();
  }

  public double getThinSideSpeed() {
    return thinSideSpeed.get();
  }

  public String getDirection() {
    return direction.get();
  }

  private DoubleLogEntry thickSideSpeedLog = new DoubleLogEntry(log, "/servo/thickSidespeed");
  private DoubleLogEntry thinSideSpeedLog = new DoubleLogEntry(log, "/servo/thinSidespeed");
  private StringLogEntry directionLog = new StringLogEntry(log, "/servo/direction");

  // SETTERS
  public void setThickSideSpeed(double speed) {
    thickSideSpeed.set(speed);
    thickSideSpeedLog.append(thickSideSpeed.get());

    servoThickSide.set(speed);
  }

  public void setThinSideSpeed(double speed) {
    thinSideSpeed.set(speed);
    thinSideSpeedLog.append(thinSideSpeed.get());

    servoThinSide.set(speed);
  }

  public void setDirection(String currentDirection) {
    direction.set(currentDirection);
    directionLog.append(getDirection());
  }
}
