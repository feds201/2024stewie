// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.controller.ToggleRumble;
import frc.robot.commands.swerve.AimToAprilTag;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToAprilTag extends SequentialCommandGroup {
  /** Creates a new AlignToAprilTag. */
  public AlignToAprilTag(CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, CommandXboxController operatorController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AimToAprilTag(drivetrain, driverController::getLeftX,
            driverController::getLeftY)
            .andThen(new ParallelCommandGroup(
                new ToggleRumble(driverController, 0.5)),
                new ToggleRumble(operatorController, 0.5)));
  }
}
