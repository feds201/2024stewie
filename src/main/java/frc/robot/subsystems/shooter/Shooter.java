// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.Map;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMain;
  private final TalonFX shooterFollower;
  private final DutyCycleEncoder shooterRotateEncoder;
  private final TalonFX shooterRotate;

  public Shooter() {
    shooterMain = new TalonFX (3);
    shooterFollower = new TalonFX (5);
    shooterRotate = new TalonFX (0);
    shooterRotateEncoder = new DutyCycleEncoder(0);
    
    shooterFollower.setControl(new StrictFollower(shooterMain.getDeviceID()));
    setupDebug();
  }

  public void rotateShooterWheels(double c_shootVelocity) {
    VelocityDutyCycle velocity = new VelocityDutyCycle(0);
    
    shooterMain.setControl(velocity.withVelocity(c_shootVelocity));
  }

  public void rotateShooterWheelsVolts(double power) {
    DutyCycleOut volts = new DutyCycleOut(0);
    
    shooterMain.setControl(volts.withOutput(power));
  }

  public void rotateShooter(double c_angle){
    PositionDutyCycle position = new PositionDutyCycle(0);
    shooterRotate.setControl(position.withPosition(c_angle));
  }
  GenericEntry f_speedGetter;
  public void setupDebug(){
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    f_speedGetter = tab.add("Max Speed", 1)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
   .getEntry();
  }

  public double getShooterSpeed(){
    return f_speedGetter.getDouble(0);
  }






  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
