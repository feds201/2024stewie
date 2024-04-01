package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;


public class swerveSlowMode extends Command {

		Double speedPercentage;

		public swerveSlowMode(Double Speed) {
				this.speedPercentage = Speed;
		}

		@Override
		public void initialize() {
				SwerveConstants.speedpercentage = speedPercentage;
		}


		@Override
		public boolean isFinished() {
				return false;
		}

		@Override
		public void end(boolean interrupted) {

		}
}
