// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.RotateWristToPosition;
import frc.robot.commands.Intake.RunIntakeWheels;
import frc.robot.commands.leds.SetLEDColor;
import frc.robot.commands.shooter.EjectNote;
import frc.robot.commands.shooter.RotateShooterToPosition;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Intake.IntakeWheels;
import frc.robot.subsystems.Intake.Wrist;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.sensors.BreakBeamSensorShooter;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.shooter.ShooterServos;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignShooterAndIntake extends ParallelCommandGroup {
  /** Creates a new AlignShooterAndIntake. */
  public AlignShooterAndIntake(ShooterRotation shooterRotation, Wrist wrist, IntakeWheels intakeWheels,
      ShooterServos servos, BreakBeamSensorShooter breakBeamSensorShooter, Leds leds) {
    addCommands(
        new RotateShooterToPosition(shooterRotation,
            () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint),
        new SequentialCommandGroup(
            new RotateWristToPosition(wrist, IntakeConstants.WristPID.kWristShooterFeederSetpoint),
            new ParallelCommandGroup(
                new RunIntakeWheels(intakeWheels, () -> IntakeConstants.kHandoffNoteWheelSpeed),
                new EjectNote(servos),
                    new SetLEDColor(leds, Leds.LedColors.GREEN))
                .until(breakBeamSensorShooter::getBeamBroken)
        // Since the beambreak is now end of motion for the note, this is not necessary
        // anymore.
        // .andThen(new ParallelDeadlineGroup(
        // new WaitCommand(ShooterConstants.kHandoffDelay),
        // new RunIntakeWheels(intakeWheels, () ->
        // IntakeConstants.kHandoffNoteWheelSpeed),
        // new EjectNote(servos),
        // new SetLEDColor(leds, Leds.LedColors.ORANGE)))
        ));
  }
}
