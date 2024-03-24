// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.EjectNote;
import frc.robot.commands.shooter.RotateShooterToPosition;
import frc.robot.commands.shooter.ShootNoteMotionMagicVelocity;
import frc.robot.subsystems.sensors.BreakBeamSensorShooter;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.shooter.ShooterServos;
import frc.robot.subsystems.shooter.ShooterWheels;
import frc.robot.subsystems.vision_sys.VisionVariables.ExportedVariables;
import frc.robot.utils.LimelightUtils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteAtSpeakerOnly extends SequentialCommandGroup {
  /**
   * Creates a new ShootNoteAtSpeakerOnly.
   * 
   * @param shooterRotation
   * @param shooterWheels
   * @param servos
   */
  public ShootNoteAtSpeakerOnly(ShooterRotation shooterRotation, ShooterWheels shooterWheels, ShooterServos servos, BreakBeamSensorShooter thing) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RotateShooterToPosition(shooterRotation,
                () -> LimelightUtils.GetSpeedAngle(
                    ExportedVariables.Distance).angle),
            new ShootNoteMotionMagicVelocity(shooterWheels,
                () -> LimelightUtils.GetSpeedAngle(ExportedVariables.Distance).speed),
            new SequentialCommandGroup(
                new WaitCommand(0.7),
                new EjectNote(servos))));
  }
}
