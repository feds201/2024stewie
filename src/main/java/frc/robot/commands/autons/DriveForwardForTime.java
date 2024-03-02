// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardForTime extends SequentialCommandGroup {
  /** Creates a new DriveForwardForTime. */
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public DriveForwardForTime(CommandSwerveDrivetrain swerve, double time) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelDeadlineGroup(
            new WaitCommand(time),
            swerve.applyRequest(
                () -> forwardStraight
                    .withVelocityX(SwerveConstants.kAutonMoveSpeed)
                    .withVelocityY(0)))
            .andThen(new ParallelDeadlineGroup(new WaitCommand(100), swerve.applyRequest(() -> brake))));
  }
}
