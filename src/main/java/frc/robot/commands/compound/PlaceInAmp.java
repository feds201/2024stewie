package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.RunIntakeWheels;
import frc.robot.commands.arm.RotateArmToPosition;
import frc.robot.commands.leds.SetLEDColor;
import frc.robot.commands.Intake.RotateWristToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeWheels;
import frc.robot.subsystems.Intake.Wrist;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.leds.Leds;

public class PlaceInAmp extends SequentialCommandGroup {
  public PlaceInAmp(Wrist wrist, IntakeWheels wheels, Arm arm, Leds leds) {
    addCommands(
        new RotateWristToPosition(wrist, IntakeConstants.WristPID.kAmpPosition),
        new SetLEDColor(leds, Leds.LedColors.BLUE),
        new ParallelCommandGroup(
            new RotateArmToPosition(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kAmpPosition),
            new SequentialCommandGroup(
                new WaitCommand(4), // sometimes even the greatest of robots have the dumbest solutions :sadge:
                new RunIntakeWheels(wheels, () -> IntakeConstants.kAmpInWheelSpeed),
                new SetLEDColor(leds, Leds.LedColors.RED))));
  }
}