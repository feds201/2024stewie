package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.RunIntakeWheels;
import frc.robot.commands.arm.RotateArmToPosition;
import frc.robot.commands.intake.amp.RotateWristToAmpi;
import frc.robot.commands.leds.SetLEDColor;
import frc.robot.commands.intake.RotateWristToPosition;
import frc.robot.commands.intake.RotateWristToPositionInfinite;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.leds.Leds;

public class PlaceInAmp extends ParallelCommandGroup {
  public PlaceInAmp(Wrist wrist, IntakeWheels wheels, Arm arm, Leds leds) {
    addCommands(
            new RotateArmToPosition(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kAmpPosition),
            new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    new RotateWristToAmpi(wrist, IntakeConstants.WristPID.kAmpPosition)
            ),
            new SequentialCommandGroup(
                    new WaitCommand(2),
                    new RunIntakeWheels(wheels, () -> IntakeConstants.kAmpInWheelSpeed)
            ));
  }
}

//        new RotateArmToPosition(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kAmpPosition),
//        new SequentialCommandGroup(
//            new WaitCommand(1),
//            new RotateWristToAmpi(wrist, IntakeConstants.WristPID.kAmpPosition)),
//        new SequentialCommandGroup(
//            new WaitCommand(3.5),
//            new RunIntakeWheels(wheels, () -> IntakeConstants.kAmpInWheelSpeed)));
//  }
//}