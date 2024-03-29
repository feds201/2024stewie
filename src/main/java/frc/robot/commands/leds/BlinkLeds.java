// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.leds.Leds;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlinkLeds extends SequentialCommandGroup {
    public BlinkLeds(Leds leds, double ledColor) {
        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(0.1), 
                new SetLEDColor(leds,Leds.LedColors.DarkGreen)),
            new ParallelDeadlineGroup(
                new WaitCommand(0.1), 
                new SetLEDColor(leds, Leds.LedColors.WHITE
            )),
            new BlinkLeds(leds, ledColor)
        );
    }
}
