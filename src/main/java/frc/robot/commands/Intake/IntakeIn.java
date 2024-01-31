// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeIn extends Command {
  /** Creates a new IntakeIn. */
  private final Intake c_intake;
  private final double c_intakeVoltage;

  public IntakeIn(Intake intake, double intakeVoltage) {
    c_intake = intake;
    c_intakeVoltage = intakeVoltage;

    addRequirements(c_intake);
      // use addRequirements( here to declare subsystem dependentc
    
  }

  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c_intake.rotateIntakeWheels(c_intakeVoltage);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_intake.rotateIntakeWheels(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
