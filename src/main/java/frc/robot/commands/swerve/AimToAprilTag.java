package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision_sys.VisionVariables;

public class AimToAprilTag extends Command {
    private final NetworkTable table = NetworkTableInstance.getDefault()
            .getTable(CameraConstants.BackCam.BACK_CAMERA_NETWORK_TABLES_NAME);

    // public static PIDController rotationPID = createPIDController();
    private final CommandSwerveDrivetrain c_swerve;
    private final PIDController c_pid;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final DoubleSupplier c_leftX, c_leftY;

    public AimToAprilTag(CommandSwerveDrivetrain swerve, DoubleSupplier leftX, DoubleSupplier leftY) {
        c_swerve = swerve;
        c_leftX = leftX;
        c_leftY = leftY;

        c_pid = new PIDController(0.025, .05, .00);
        c_pid.setTolerance(.25, 0.05); // allowable angle error
        c_pid.enableContinuousInput(0, 360); // it is faster to go 1 degree from 359 to 0 instead of 359 degrees
        c_pid.setIntegratorRange(-0.3  , 0.3);
        c_pid.setSetpoint(-10); // 0 = apriltag angle
        Shuffleboard.getTab("swerve").add("april tag pid", c_pid);
        
        addRequirements(swerve);
    }

    public void initialize() {

    }

    public boolean isFinished() {
        return false;
    }

    public void execute() {
        c_swerve.setControl(drive
              .withVelocityX(-c_leftX.getAsDouble()
                  * SwerveConstants.MaxSpeed)
              .withVelocityY(-c_leftY.getAsDouble()
                  * SwerveConstants.MaxSpeed)
              .withRotationalRate(c_pid.calculate(VisionVariables.BackCam.target.getX())));
      
    }

    public void end(boolean interrupted) {

    }
}
