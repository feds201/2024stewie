package frc.robot.commands.vision;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Grab_Note extends Command {

    public Grab_Note() {
    }

    public void initialize() {
        // Replace 'fieldX', 'fieldY', and 'fieldID' with the actual field names
        SmartDashboard.putNumber("X-axis", 12345678);
        SmartDashboard.putNumber("Y-axis", 12345678);
        SmartDashboard.putNumber("Target ID", 1);
    }

    public void execute() {
        SmartDashboard.putBoolean("Executing", true);

    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {}
}