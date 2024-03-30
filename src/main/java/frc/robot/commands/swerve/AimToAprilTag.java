package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionVariables;
import frc.robot.utils.LimelightUtils;

public class AimToAprilTag extends Command {
        private final CommandSwerveDrivetrain c_swerve;
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(SwerveConstants.MaxSpeed * 0.1)
                .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final DoubleSupplier c_leftX, c_leftY;
        // private double startTime;
        // private final double timeout = 2.0; // Time limit in seconds
        // private final double rangeTolerance = 0.3; // Range within which to consider aligned
        private double lastOutput = 1;
        // private double axisofinit;
        private DoubleSupplier c_limelightDistance;

        public AimToAprilTag(CommandSwerveDrivetrain swerve, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier limelightDistance) {
                c_swerve = swerve;
                c_leftX = leftX;
                c_leftY = leftY;
                c_limelightDistance = limelightDistance;

                addRequirements(swerve);
        }

		public void initialize() {
                SmartDashboard.putBoolean("AimToAPerilTagCommand", true);
                c_swerve.resetPID();
                c_swerve.setTarget(LimelightUtils.MapDistanceToOffset(c_limelightDistance.getAsDouble()));
        }

        public boolean isFinished() {
                // Check for PID at setpoint or timeout
                return c_swerve.getPIDAtSetpoint();// || Math.abs(lastOutput) < SwerveConstants.kAlignmentOutput;

        }

        public void execute() {


                double output = c_swerve.getPIDRotation(VisionVariables.BackCam.target.getX());

                SmartDashboard.putNumber("errorVal", VisionVariables.BackCam.target.getX());
                SmartDashboard.putNumber("Output", output);


                c_swerve.setControl(drive
                        .withVelocityX(-c_leftX.getAsDouble() * SwerveConstants.MaxSpeed)
                        .withVelocityY(-c_leftY.getAsDouble() * SwerveConstants.MaxSpeed)
                        .withRotationalRate(output));
                lastOutput = output;

				if (Math.abs(output) <= 2)  {
						SmartDashboard.putBoolean("AimToAPerilTagCommand", false);
				}
        }

		public void end(boolean interrupted) {
		        SmartDashboard.putBoolean("AimToAPerilTagCommand", false);
		        // Additional logic for timeout or completion here if needed
		}
}


