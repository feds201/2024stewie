package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Grab_Note extends Command {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-notes");
    private final VisionSubsystem vision_sys;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final CommandSwerveDrivetrain drivetrain;

    public Grab_Note(VisionSubsystem visionSys, CommandSwerveDrivetrain drivetrain1) {
        this.vision_sys = visionSys;
        this.drivetrain = drivetrain1;
        this.addRequirements(drivetrain);
        this.addRequirements(vision_sys);
    }



    public void execute() {
        while (true) {
            Translation2d Cords = vision_sys.findNote();
            SmartDashboard.putNumber("X", Cords.getX());
            SmartDashboard.putNumber("Y", Cords.getY());
            double area = table.getEntry("area").getDouble(0);
            if (area > 16) {
                drivetrain.applyRequest(() -> brake);
                break;
            }
        }

        //TODO End loop when note is within a center
        // TODO and area is X % of the screen

    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {

    }
}