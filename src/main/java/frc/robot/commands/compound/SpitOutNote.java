// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.commands.intake.RotateWristToPosition;
import frc.robot.commands.intake.RunIntakeWheels;
import frc.robot.constants.IntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpitOutNote extends SequentialCommandGroup {
  /** Creates a new SpitOutNote. */
  public SpitOutNote(Wrist wrist, IntakeWheels wheels) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateWristToPosition(wrist, IntakeConstants.WristPID.kSpitOutPosition),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new RunIntakeWheels(wheels, () -> IntakeConstants.kSpitOutNoteWheelSpeed))
    );
  }
}
