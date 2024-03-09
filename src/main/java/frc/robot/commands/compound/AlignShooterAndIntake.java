// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.RunIntakeWheels;
import frc.robot.commands.intake.RotateWristPID;
import frc.robot.commands.shooter.EjectNote;
import frc.robot.commands.shooter.RotateShooter;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.sensors.BreakBeamSensorShooter;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.shooter.ShooterServos;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignShooterAndIntake extends ParallelCommandGroup {
  /** Creates a new AlignShooterAndIntake. */
  public AlignShooterAndIntake(ShooterRotation shooterRotation, Wrist wrist, IntakeWheels intakeWheels, ShooterServos servos, BreakBeamSensorShooter breakBeamSensorShooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            new RotateShooter(shooterRotation,
                () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint),
            new RotateWristPID(wrist, IntakeConstants.WristPID.kWristShooterFeederSetpoint)
                .andThen(
                    new WaitCommand(0.75)
                        .andThen(
                            new ParallelCommandGroup(
                                new RunIntakeWheels(intakeWheels, () -> IntakeConstants.kHandoffNoteWheelSpeed),
                                new EjectNote(servos))
                                .until(breakBeamSensorShooter::getBeamBroken)
                                .andThen(new ParallelDeadlineGroup(
                                    new WaitCommand(ShooterConstants.kHandoffDelay),
                                    new RunIntakeWheels(intakeWheels, () -> IntakeConstants.kHandoffNoteWheelSpeed),
                                    new EjectNote(servos))))));
  }
}
