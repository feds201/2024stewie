package frc.robot.Constants;

public class CameraConstants {
    public static int MaxSpeed = 1;
    public static int MaxAngularRate = 1;

    public static class BackCam {
        // private static final String discription = "For finding AprilTags and for
        // distance calculation";
        public static final String kname = "BackCamera";
        public static final String ip = "http://10.2.1.17:5801";
        public static final String BACK_CAMERA_NETWORK_TABLES_NAME = "limelight-april";
        public static final double APRILTAG_HEIGHT = 0.0;
        public static final double CAMERA_HEIGHT = 0.0;
        public static double horizontal_fov = 59.6;
        public static double vertical_fov = 45.7;
        public static int CameraWidth = 640;
        public static int CameraHeight = 480;
    }

    public static class FrontCam {
        // private static final String description = "For finding AprilTags and going
        // towards them";
        public static final String kName = "FrontCamera";
        public static final String ip = "http://10.2.1.43:5801";
        public static final String FRONT_CAMERA_NETWORK_TABLES_NAME = "limelight-notes";
        public static double horizontal_fov = 63.3;
        public static double vertical_fov = 49.7;
        public static int CameraWidth = 640;
        public static int CameraHeight = 480;


    }
}