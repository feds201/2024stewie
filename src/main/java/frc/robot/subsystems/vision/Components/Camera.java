package frc.robot.subsystems.vision.Components;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


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
        System.out.println("Camera instance created");
    }

    public boolean checkNote() {
        return Variables.BackCam.tv == 1;
    }



}
