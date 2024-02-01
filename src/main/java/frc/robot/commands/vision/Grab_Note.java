package frc.robot.commands.vision;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Grab_Note extends Command {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-notes");
    
    public Grab_Note() {
        
    }

    public void initialize() {
        
    }

    public void execute() {
        double xAxis = table.getEntry("tx").getNumber(0).doubleValue();
        //TODO End loop when note is within a center
        // TODO and area is X % of the screen

    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {

    }
}