package frc.robot.subsystems.vision.Components;

import edu.wpi.first.math.geometry.Translation2d;

public class Camera {
    public static BackCamera backCam;
    public static FrontCamera frontCam;


    public Camera() {
        backCam = new BackCamera();// For AprilTag
        frontCam = new FrontCamera();// For Notes
    }



    public void periodic() {
        backCam.periodic();// Runs the periodic method in BackCamera
        frontCam.periodic();// Runs the periodic method in FrontCamera
        
    }

    public boolean checkNote() {
        return backCam.checkNote();
    }



    public Translation2d getNoteTargetLocation() {
        return frontCam.getNoteTargetLocation();
    }



    public double[] getVelocityForNote() {
        return frontCam.getVelocity(frontCam.getNoteTargetLocation());
    }
}