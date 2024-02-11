package frc.robot.constants;

public class CameraConstants {
    public static int MaxSpeed = 1;
    public static int MaxAngularRate = 1;

    public static class BackCam {
        // private static final String discription = "For finding AprilTags and for
        // distance calculation";
        public static final String kname = "BackCamera";
        public static final String ip = "http://10.2.1.17:5801";
        public static final String kBackCameraNetworkTablesName = "limelight-april";
        public static double horizontal_fov = 62.5;
        public static int CameraWidth = 640;
        public static int CameraHeight = 480;
    }

    public static class FrontCam {
        // private static final String discription = "For finding AprilTags and going
        // towards them";
        public static final String kName = "FrontCamera";
        public static final String ip = "http://10.2.1.43:5801";
        public static final String kFrontCameraNetworkTablesName = "limelight-notes";
        public static double horizontal_fov = 62.5;
        public static int CameraWidth = 640;
        public static int CameraHeight = 480;

    }
}