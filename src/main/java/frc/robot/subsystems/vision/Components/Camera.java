package frc.robot.subsystems.vision.Components;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    public static BackCamera backCam;
    public static FrontCamera frontCam;

    public static class Variables {
        public static class FrontCam {
            private static final String discription = "For finding AprilTags and for distance calculation";
            public static final String name = "FrontCamera";
            public static final String ip = "http://10.2.1.17:5801";
            public static int tv;
            public static double tx;
            public static double ty;
            public static double ta;
            public static int tid;
            public static Number CameraMode;

        }

        public static class BackCam {
            private static final String discription = "For finding AprilTags and going towards them";
            public static final String name = "FrontCamera";
            public static final String ip = "http://10.2.1.43:5801";
            public static int tv;
            public static double tx;
            public static double ty;
            public static double ta;
            public static int tid;
            public static Number CameraMode;

        }
    }
    public Camera() {
        backCam = new BackCamera();
        frontCam = new FrontCamera();
        SmartDashboard.putString(Variables.BackCam.name, "Hello");
    }

    public boolean checkNote() {
        double hello = NetworkTableInstance.getDefault().getTable("limelight-notes").getEntry("tv").getNumber(0).doubleValue();
        return hello == 1;
    }

    public void periodic() {
        backCam.BackCamPeroidic();
        frontCam.FrontCamPeroidic();
    }
}