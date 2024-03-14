package frc.robot.commands.compound;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Intake.RunIntakeWheels;
import frc.robot.commands.arm.RotateArm;
import frc.robot.commands.Intake.RotateWristPID;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeWheels;
import frc.robot.subsystems.Intake.Wrist;
import frc.robot.subsystems.arm.Arm;

public class PlaceInAmp extends SequentialCommandGroup {
		public PlaceInAmp(Wrist wrist, IntakeWheels wheels, Arm arm) {
				// TODO: Add your sequential commands in the super() call, e.g.
				//           super(new OpenClawCommand(), new MoveArmCommand());
				super();
                addCommands(
                        new RotateWristPID(wrist, IntakeConstants.WristPID.kAmpPosition),
                        new RotateArm(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kAmpPosition),
                        new RunIntakeWheels(wheels, () -> IntakeConstants.kAmpInWheelSpeed)
                );
		}
}