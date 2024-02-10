package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.vision.Components.Camera;

public class VisionSubsystem extends SubsystemBase {
    private final Camera camera;

    public VisionSubsystem() {
        camera = new Camera();
        System.out.println("VisionSubsystem initiated");
    }

    public void periodic() {

        camera.periodic();
        boolean targetLocked = checkNote();// If the target is locked, it will put the X and Y cords of the target on the SmartDashboard
        if (targetLocked) {
            VisionVariables.ExportedVariables.Velocity = camera.getVelocityForNote();

        }
        
    }


    public boolean checkNote() {
        return camera.checkNote();
    }
}