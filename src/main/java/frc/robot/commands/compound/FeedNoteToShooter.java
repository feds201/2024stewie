// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.RotateWristPIDInfinite;
import frc.robot.commands.arm.RotateArm;
import frc.robot.commands.shooter.RotateShooter;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Intake.IntakeWheels;
import frc.robot.subsystems.Intake.Wrist;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.ShooterRotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedNoteToShooter extends ParallelCommandGroup {
  /** Creates a new FeedNoteToShooter. */
  public FeedNoteToShooter(ShooterRotation shooterRotation, Wrist wrist, Arm arm, IntakeWheels wheels) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RotateArm(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint), //Rotate arm 2.0 degrees
        new SequentialCommandGroup( 
            new WaitCommand(ArmConstants.kArmRotationDelay), //Wait 3 seconds
            new ParallelCommandGroup(
                new RotateWristPIDInfinite(wrist, IntakeConstants.kWristShooterFeederSetpoint), //Rotate Intake to the setpoint (7) (90 degrees)
                new RotateShooter(shooterRotation,
                    () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint), //PROBLEM (NOT ROTATING)
                new SequentialCommandGroup(
                    new WaitCommand(ShooterConstants.kRotateShooterDelay),
                    new ParallelDeadlineGroup(
                        new IntakeIn(wheels, () -> IntakeConstants.kWheelSpeed))))));
  }
}
