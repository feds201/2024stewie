package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.RunIntakeWheels;
import frc.robot.commands.shooter.RotateShooterToPosition;
import frc.robot.commands.arm.RotateArmToPosition;
import frc.robot.commands.intake.RotateWristInfiniteWithAmpPID;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.ShooterRotation;

public class PlaceInAmp extends ParallelCommandGroup {
  public PlaceInAmp(Wrist wrist, IntakeWheels wheels, Arm arm, Leds leds, ShooterRotation shooterRotation) {
    addCommands(
            new RotateArmToPosition(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kAmpPosition),
            new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    new RotateWristInfiniteWithAmpPID(wrist, IntakeConstants.WristPID.kAmpPosition) 
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