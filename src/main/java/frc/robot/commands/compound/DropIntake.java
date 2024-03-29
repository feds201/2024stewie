// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.RotateWristToPosition;
import frc.robot.commands.arm.RotateArmToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropIntake extends SequentialCommandGroup {
    /** Creates a new DeployIntake. */
    public DropIntake(Wrist wrist, frc.robot.subsystems.arm.Arm arm) {

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
              new ParallelCommandGroup(
                  new RotateWristToPosition(wrist, IntakeConstants.WristPID.kWristNotePosition),
                new RotateArmToPosition(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint)
              ));
      
    }
}
