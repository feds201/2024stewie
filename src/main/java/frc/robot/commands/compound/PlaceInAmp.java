package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.RunIntakeWheels;
import frc.robot.commands.arm.RotateArmToPosition;
import frc.robot.commands.leds.SetLEDColor;
import frc.robot.commands.Intake.RotateWristToPosition;
import frc.robot.commands.Intake.RotateWristToPositionInfinite;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeWheels;
import frc.robot.subsystems.Intake.Wrist;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.leds.Leds;

public class PlaceInAmp extends ParallelCommandGroup {
  public PlaceInAmp(Wrist wrist, IntakeWheels wheels, Arm arm, Leds leds) {
    addCommands(
            new RotateArmToPosition(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kAmpPosition),
            new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    new RotateWristToPositionInfinite(wrist, IntakeConstants.WristPID.kAmpPosition)
            ),
            new SequentialCommandGroup(
                    new WaitCommand(3.2),
                    new RunIntakeWheels(wheels, () -> IntakeConstants.kAmpInWheelSpeed)
            ));
  }
}

//        new RotateArmToPosition(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kAmpPosition),
//        new SequentialCommandGroup(
//            new WaitCommand(1),
//            new RotateWristToPositionInfinite(wrist, IntakeConstants.WristPID.kAmpPosition)),
//        new SequentialCommandGroup(
//            new WaitCommand(3.5),
//            new RunIntakeWheels(wheels, () -> IntakeConstants.kAmpInWheelSpeed)));
//  }
//}