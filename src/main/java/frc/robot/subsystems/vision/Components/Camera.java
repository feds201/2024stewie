package frc.robot.subsystems.vision.Components;

import edu.wpi.first.math.geometry.Translation2d;

public class Camera {
    public static BackCamera backCam;
    public static FrontCamera frontCam;


    public Camera() {
        backCam = new BackCamera();
        frontCam = new FrontCamera();
    }



    public void periodic() {
        backCam.periodic();
        frontCam.periodic();
    }

    public boolean checkNote() {
        return backCam.checkNote();
    }



    public Translation2d getTargetLocation() {
        return backCam.getTargetLocation();
    }
}