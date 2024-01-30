package frc.robot.subsystems.vision.Components;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public class Variables {


        public static class FrontCam {
            private static final String discription = "For finding AprilTags and for distance calculation";
            public static final String name = "FrontCamera";
            public static final String ip = "";
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
            public static final String ip = "";
            public static int tv;
            public static double tx;
            public static double ty;
            public static double ta;
            public static int tid;
            public static Number CameraMode;

        }
    }
    public Camera() {
    }


}
