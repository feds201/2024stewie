// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMain;
  private final TalonFX shooterFollower;
  // private final DutyCycleEncoder shooterRotateEncoder;
  private final TalonFX shooterRotate;


  private GenericEntry voltageSlider;
  private GenericEntry velocitySlider;
  private GenericEntry shooterAngle;
  private ShuffleboardTab shooterTab;

  public Shooter() {
 
    shooterMain = new TalonFX(3);
    shooterFollower = new TalonFX(5);
    shooterRotate = new TalonFX(0);
    // shooterRotateEncoder = new DutyCycleEncoder(9);

   
    // shooterFollower.getConfigurator().apply(configs);
      SignalLogger.start();
    SignalLogger.setPath("/media/sda1/ctre-logs/");

    shooterFollower.setControl(new StrictFollower(shooterMain.getDeviceID()));
    setupDebug();
  //   shooterRotateEncoder.reset();
  }

  public void rotateShooterWheels(double c_shootVelocity) {
    VelocityDutyCycle velocity = new VelocityDutyCycle(0);
    shooterMain.setControl(velocity.withVelocity(c_shootVelocity));
  }

  public void rotateShooterWheelsVolts(double power) {
    DutyCycleOut volts = new DutyCycleOut(0);
    shooterMain.setControl(volts.withOutput(-power));
  }

  public void rotateShooterWheelsVelocity(double velocity) {
    VelocityDutyCycle velocitySupplier = new VelocityDutyCycle(0);
    shooterMain.setControl(velocitySupplier.withVelocity(velocity));
  }

  public void rotateShooter(double voltage) {
    DutyCycleOut angleVolts = new DutyCycleOut(0);
    shooterRotate.setControl(angleVolts.withOutput(voltage));
  }

  public void setupDebug() {
    shooterTab = Shuffleboard.getTab("Shooter");
    voltageSlider = shooterTab.add("Voltage", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        .getEntry();
    shooterAngle = shooterTab.add("Shooter Angle", 0)
       .withWidget(BuiltInWidgets.kNumberSlider)
       .withProperties(Map.of("min", 0, "max", 80))
       .getEntry();

    velocitySlider = shooterTab.add("Velocity", 0)
        // .withWidget(BuiltInWidgets.kNumberSlider)
        // .withProperties(Map.of("min", 0, "max", 0.1)) // specify widget properties here
        .getEntry();
  }

  public double getShooterAngle(){
    return shooterAngle.getDouble(0);
  } 

  public double getShooterVoltage() {
    return voltageSlider.getDouble(0);
  }

  public double getShooterVelocity() {
    return velocitySlider.getDouble(0);
  }

  public double getShooterEncoderPosition() {
    return shooterRotateEncoder.getAbsolutePosition();
  }


  private void configMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    configs.Slot0.kP = 0.0; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; //An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.000; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / Rotation per second
// configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
//     configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
//     configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
//     configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
//                              // volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    // /*
    //  * Torque-based velocity does not require a feed forward, as torque will
    //  * accelerate the rotor up to the desired velocity by itself
    //  */
    // configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    // configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    // configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    shooterMain.getConfigurator().apply(configs);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
