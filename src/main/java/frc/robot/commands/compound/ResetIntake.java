// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.RotateWristPID;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetIntake extends ParallelCommandGroup {
  /** Creates a new ResetIntake. */
  public ResetIntake(Wrist wrist, IntakeWheels intakeWheels) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RotateWristPID(wrist, IntakeConstants.kWristIdlePosition),
        new IntakeIn(intakeWheels, () -> 0));
  }
}
