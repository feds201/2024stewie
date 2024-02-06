package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Grab_Note extends Command {

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


        //TODO End loop when note is within a center
        // TODO and area is X % of the screen

    }

    public boolean isFinished() {
        return true;


    }

    public void end(boolean interrupted) {

    }
}