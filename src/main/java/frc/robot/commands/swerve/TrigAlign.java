package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision_sys.VisionVariables;
import frc.robot.utils.KalmanFilter1D;
import frc.robot.utils.LimelightUtils;

public class TrigAlign extends Command {
    private final CommandSwerveDrivetrain c_swerve;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final DoubleSupplier c_leftX, c_leftY;

    private final double initialEstimate = 0;
    private final double initialError = 1;
    private final double processNoise = 0.001;
    private final double measurementNoise = 0.1;

    KalmanFilter1D filter1D = new KalmanFilter1D(processNoise, measurementNoise, initialEstimate, initialError);

    public TrigAlign(CommandSwerveDrivetrain swerve, DoubleSupplier leftX, DoubleSupplier leftY) {
        c_swerve = swerve;
        c_leftX = leftX;
        c_leftY = leftY;

        addRequirements(swerve);
    }

    public void initialize() {
        SmartDashboard.putBoolean("AimToAPrilTagCommand", true);
        c_swerve.resetPID();
        c_swerve.setTarget(LimelightUtils.getTrigAlignAngle(filter1D.filter(VisionVariables.BackCam.target.getDistance()), filter1D.filter(VisionVariables.BackCam.target.getDistance())));
    }

    public boolean isFinished() {
        return true;
    }

    public void execute() {
        c_swerve.setControl(drive
                .withVelocityX(-c_leftX.getAsDouble() * SwerveConstants.MaxSpeed)
                .withVelocityY(-c_leftY.getAsDouble() * SwerveConstants.MaxSpeed)
                .withRotationalRate(0.5));
    }

    public void end(boolean interrupted) {

    }
}


