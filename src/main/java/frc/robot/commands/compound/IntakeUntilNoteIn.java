// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.RunIntakeWheels;
import frc.robot.commands.leds.SetLEDColor;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeWheels;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.sensors.BreakBeamSensorIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeUntilNoteIn extends SequentialCommandGroup {
  /** Creates a new RotateUntilNoteIn. */
  public IntakeUntilNoteIn(IntakeWheels intakeWheels, BreakBeamSensorIntake irSensor, Leds leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RunIntakeWheels(intakeWheels, () -> IntakeConstants.kIntakeNoteWheelSpeed)
            .until(() -> irSensor.getBeamBroken()),
        new SetLEDColor(leds, Leds.LedColors.YELLOW),
        new ParallelDeadlineGroup(
            new WaitCommand(IntakeConstants.kDistanceSensorDetectedDelay), // This should not be necessary
            new RunIntakeWheels(intakeWheels, () -> IntakeConstants.kIntakeNoteWheelSpeed)));
  }
}
