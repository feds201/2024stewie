package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision_sys.VisionVariables;
import frc.robot.utils.LimelightUtils;

public class AlignWithNOteCommand extends Command {
        private final CommandSwerveDrivetrain c_swerve;
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(SwerveConstants.MaxSpeed * 0.1)
                .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final DoubleSupplier c_leftX, c_leftY;
        private double startTime;
        private final double timeout = 2.0; // Time limit in seconds
        private final double rangeTolerance = 3; // Range within which to consider aligned
        private double lastOutput = 1;
        private double axisofinit;
        private DoubleSupplier c_limelightDistance;

        public AlignWithNOteCommand(CommandSwerveDrivetrain swerve, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier limelightDistance) {
                c_swerve = swerve;
                c_leftX = leftX;
                c_leftY = leftY;
                c_limelightDistance = limelightDistance;

                addRequirements(swerve);
        }

		public AlignWithNOteCommand(CommandSwerveDrivetrain cSwerve, DoubleSupplier cLeftX, DoubleSupplier cLeftY) {
				c_swerve = cSwerve;
				c_leftX = cLeftX;
				c_leftY = cLeftY;
		}

		public void initialize() {
                SmartDashboard.putBoolean("AlignWithNote", true);
                c_swerve.resetPID();
                c_swerve.setTarget(LimelightUtils.MapDistanceToOffset(c_limelightDistance.getAsDouble()));
        }

        public boolean isFinished() {
                // Check for PID at setpoint or timeout
                return c_swerve.getPIDAtSetpoint();// || Math.abs(lastOutput) < SwerveConstants.kAlignmentOutput;

        }

        public void execute() {

                double output = c_swerve.getPIDRotation(VisionVariables.FrontCam.target.getX());

                SmartDashboard.putNumber("errorVal", VisionVariables.BackCam.target.getX());
                SmartDashboard.putNumber("Output", output);


                c_swerve.setControl(drive
                        .withVelocityX(-c_leftX.getAsDouble() * SwerveConstants.MaxSpeed)
                        .withVelocityY(-c_leftY.getAsDouble() * SwerveConstants.MaxSpeed)
                        .withRotationalRate(output));
                lastOutput = output;

                if (Math.abs(output) <= 0.37)  {
                        SmartDashboard.putBoolean("AlignWithNote", false);
                }
        }

        public void end(boolean interrupted) {
                SmartDashboard.putBoolean("AlignWithNote", false);
                // Additional logic for timeout or completion here if needed
        }
}


