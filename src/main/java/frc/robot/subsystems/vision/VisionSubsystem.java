package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Components.Camera;

public class VisionSubsystem extends SubsystemBase {
    private final Camera camera;

    public VisionSubsystem() {
        camera = new Camera();
        System.out.println("VisionSubsystem initiated");
    }


    public void periodic() {
        // Check for Note
        if (camera.checkNote()){
            SmartDashboard.putBoolean("Target Locked", true);
        }else {
            SmartDashboard.putBoolean("Target Locked", false);
        }

        double xAxis = NetworkTableInstance.getDefault().getTable("limelight-notes").getEntry("tx").getNumber(0).doubleValue();
        double yAxis = NetworkTableInstance.getDefault().getTable("limelight-notes").getEntry("ty").getNumber(0).doubleValue();

        SmartDashboard.putBoolean("Executing", true);
        SmartDashboard.putNumber("X", xAxis);
        if (xAxis < -3){
            SmartDashboard.putString("Direction", "Right");
        }
        else if (xAxis > 3){
            SmartDashboard.putString("Direction", "Left");
        }else {
            SmartDashboard.putString("Direction", "Center");
        }



    }
}