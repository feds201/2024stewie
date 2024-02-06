package frc.robot.subsystems.vision.Components;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.Constants.Variables;

public class Camera {
    public static BackCamera backCam;
    public static FrontCamera frontCam;


    public Camera() {
        backCam = new BackCamera();
        frontCam = new FrontCamera();
        SmartDashboard.putString(Variables.BackCam.name, "Hello");
    }



    public void periodic() {
    }

    public boolean checkNote() {
        return backCam.checkNote();
    }
}