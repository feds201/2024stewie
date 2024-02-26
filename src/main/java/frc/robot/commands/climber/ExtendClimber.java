// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;


public class ExtendClimber extends Command {
  /** Creates a new ExtendArm. */
  private final Climber c_climber;
  private final DoubleSupplier c_voltage;
  
  public ExtendClimber(Climber climber, DoubleSupplier voltage) {
    c_climber = climber;
    c_voltage = voltage;
    addRequirements(c_climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c_climber.setClimberVoltage(c_voltage.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_climber.setClimberVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
