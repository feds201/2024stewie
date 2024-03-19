package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision_sys.VisionVariables;

public class AimToAprilTag extends Command {
    private final CommandSwerveDrivetrain c_swerve;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final DoubleSupplier c_leftX, c_leftY;
    private double startTime;
    private final double timeout = 2.0; // Time limit in seconds
    private final double rangeTolerance = 0.3; // Range within which to consider aligned
    private double lastOutput = 1;
    private double axisofinit;

    public AimToAprilTag(CommandSwerveDrivetrain swerve, DoubleSupplier leftX, DoubleSupplier leftY) {
        c_swerve = swerve;
        c_leftX = leftX;
        c_leftY = leftY;

        addRequirements(swerve);
    }

    public void initialize() {
        c_swerve.resetPID();


    }

    public boolean isFinished() {
        // Check for PID at setpoint or timeout
        return c_swerve.getPIDAtSetpoint();// || Math.abs(lastOutput) < SwerveConstants.kAlignmentOutput;
    }

    public void execute() {
        axisofinit =  (double) Math.round(VisionVariables.BackCam.target.getX() * 100) /100;

        double output = c_swerve.getPIDRotation(axisofinit);

        SmartDashboard.putNumber("alignment/output", output);

        // // Additional check for being within range
        // if (Math.abs(VisionVariables.BackCam.target.getX()) < rangeTolerance) {
        //     isFinished(); // Force completion if within range
        // } else {
            c_swerve.setControl(drive
                    .withVelocityX(-c_leftX.getAsDouble()
                            * SwerveConstants.MaxSpeed)
                    .withVelocityY(-c_leftY.getAsDouble()
                            * SwerveConstants.MaxSpeed)
                    .withRotationalRate(output));

              lastOutput = output;      
        // }
    }

    public void end(boolean interrupted) {
        // Additional logic for timeout or completion here if needed
    }
}
