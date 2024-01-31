package frc.robot.subsystems.vision;

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
        SmartDashboard.putBoolean("Target Locked", camera.checkNote());


    }
}