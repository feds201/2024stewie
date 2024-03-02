// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.RotateWristPID;
import frc.robot.commands.controller.ToggleRumble;
import frc.robot.commands.shooter.RotateShooter;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeWheels;
import frc.robot.subsystems.Intake.Wrist;
import frc.robot.subsystems.sensors.BreakBeamSensorIntake;
import frc.robot.subsystems.shooter.ShooterRotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeployIntake extends SequentialCommandGroup {
  /** Creates a new DeployIntake. */
  public 
  DeployIntake(Wrist wrist, IntakeWheels intakeWheels, ShooterRotation shooterRotation,
      BreakBeamSensorIntake breakBeamSensorIntake) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelDeadlineGroup(
            new RotateWristPID(wrist, IntakeConstants.kWristNotePosition),
            // new RotateShooter(shooterRotation, () -> -5),
            new IntakeIn(intakeWheels, () -> -0.5)),
        new IntakeIn(intakeWheels, () -> -0.5)
            .until(breakBeamSensorIntake::getBeamBroken),
        new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(0.14),
                new IntakeIn(intakeWheels, () -> -0.5)),
            new ParallelDeadlineGroup(
                new RotateWristPID(wrist,
                    IntakeConstants.kWristIdlePosition),
                new IntakeIn(intakeWheels, () -> 0))));
  }
}
