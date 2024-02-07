package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Grab_Note extends Command {
    private final VisionSubsystem vision_sys;
    private final CommandSwerveDrivetrain swerve;
    private final double MaxSpeed = 6;
    private final double MaxAngularRate = 1.5 * Math.PI;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Grab_Note(VisionSubsystem visionSys, CommandSwerveDrivetrain swerve) {
        this.vision_sys = visionSys;
        this.swerve = swerve;
        this.addRequirements(this.vision_sys);
        this.addRequirements(this.swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Executing", true);
        Translation2d noteCords = vision_sys.getTargetLocation();
        String direction = getDirection(noteCords);
        SmartDashboard.putString("Direction", direction);
        // TODO Move the robot in the direction of the note
        // TODO Use the swerve drive to move the robot
        // TODO Use the direction to determine the direction of the robot

        double[] velocity = getVelocity(noteCords);
        // swerve.applyRequest(() -> drive.withVelocityX(velocity[0])
        //         .withVelocityY(velocity[1])
        //         .withRotationalRate(velocity[2]));
        swerve.applyRequest(() -> 
                drive.withVelocityX(1)
                .withVelocityY(0)
                .withRotationalRate(0));

        // TODO End loop when note is within a center
        // TODO and area is X % of the screen

    }

    private String getDirection(Translation2d noteCords) {
        double deadband = 0.1; // Adjust the deadband value as needed
        double x = noteCords.getX();

        if (Math.abs(x) < deadband) {
            return "Center";
        } else if (x > 0) {
            return "Right";
        } else {
            return "Left";
        }
    }

    private double[] getVelocity(Translation2d noteCords) {
        double deadband = 0.1; // Adjust the deadband value as needed
        double x = noteCords.getX();
        double y = noteCords.getY();
        double[] velocity = new double[3];

        if (Math.abs(x) < deadband) {
            velocity[0] = 0;
            velocity[1] = 0;
            velocity[2] = 0;
        } else if (x > 0) {
            velocity[0] = -0.5 * MaxSpeed;
            velocity[1] = 0.5 * MaxSpeed;
            velocity[2] = -0.5 * MaxAngularRate;
        } else {
            velocity[0] = 0.5 * MaxSpeed;
            velocity[1] = -0.5 * MaxSpeed;
            velocity[2] = 0.5 * MaxAngularRate;
        }
        SmartDashboard.putNumber("X", velocity[0]);
        SmartDashboard.putNumber("Y", velocity[1]);
        SmartDashboard.putNumber("Rotational", velocity[2]);

        return velocity;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Executing", false);

        swerve.applyRequest(() -> drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }
}