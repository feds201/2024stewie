// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final CANSparkMax climberMain;
  private final CANSparkMax climberFollower;
 
  /** Creates a new Climber. */
  public Climber() {
   
    climberMain = new CANSparkMax(0,MotorType.kBrushless);
    climberFollower = new CANSparkMax(0,MotorType.kBrushless);
 
        climberFollower.follow(climberMain);

  }
public void climberMainSpeed (double c_climberVoltage){
  climberMain.set(c_climberVoltage);
}


  @Override
  public void periodic() {
  
    // This method will be called once per scheduler run
  }
}
