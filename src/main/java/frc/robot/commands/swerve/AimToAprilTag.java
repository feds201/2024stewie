package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

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
        return c_swerve.getPIDAtSetpoint();
    }

    public void execute() {
        c_swerve.setControl(drive
              .withVelocityX(-c_leftX.getAsDouble()
                  * SwerveConstants.MaxSpeed)
              .withVelocityY(-c_leftY.getAsDouble()
                  * SwerveConstants.MaxSpeed)
              .withRotationalRate(c_swerve.getPIDRotation(VisionVariables.BackCam.target.getX())));
      
    }

    public void end(boolean interrupted) {

    }
}
