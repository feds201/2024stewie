package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.EjectNote;
import frc.robot.commands.shooter.RotateShooterToPosition;
import frc.robot.commands.shooter.ShootNoteMotionMagicVelocity;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIRSensor;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.shooter.ShooterServos;
import frc.robot.subsystems.shooter.ShooterWheels;

public class ShootAcrossField extends SequentialCommandGroup {
		public ShootAcrossField(ShooterRotation shooterRotation, ShooterWheels shooterWheels, ShooterServos servos, ShooterIRSensor breaker) {
			addCommands(
					new RotateShooterToPosition(shooterRotation, () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint),
					new ShootNoteMotionMagicVelocity(shooterWheels, () -> 100.00,() -> 100.00),
					new WaitCommand(0.7),
					new EjectNote(servos).until(breaker::getBeamBroken)
			);
		}
}
