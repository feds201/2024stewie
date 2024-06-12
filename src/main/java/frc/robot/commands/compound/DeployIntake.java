// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.RunIntakeWheels;
import frc.robot.commands.shooter.RotateShooterToPosition;
import frc.robot.commands.intake.RotateWristToPosition;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.IntakeIRSensor;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.ShooterRotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeployIntake extends SequentialCommandGroup {
    public DeployIntake(Wrist wrist, IntakeWheels intakeWheels, ShooterRotation shooterRotation,
                        IntakeIRSensor breakBeamSensorIntake, Leds leds, CommandXboxController driver, CommandXboxController operator) {
        addCommands(
            new ParallelDeadlineGroup(
                new RotateWristToPosition(wrist, IntakeConstants.WristPID.kWristNotePosition),
                new RunIntakeWheels(intakeWheels, () -> IntakeConstants.kIntakeNoteWheelSpeed),
                new RotateShooterToPosition(shooterRotation,
                    () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint)),
            new IntakeUntilNoteIn(intakeWheels, breakBeamSensorIntake, leds, driver, operator),
//            new SetLEDColor(leds, Leds.LedColors.RED),
//            new RotateShooterToPosition(shooterRotation,
//                () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint),
            new ParallelDeadlineGroup(
                new RotateWristToPosition(wrist,
                    IntakeConstants.WristPID.kWristShooterFeederSetpoint),
                new RunIntakeWheels(intakeWheels, () -> 0),
                new RotateShooterToPosition(shooterRotation,
                    () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint)));
    }
}
