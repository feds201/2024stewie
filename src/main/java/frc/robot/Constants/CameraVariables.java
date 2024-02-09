package frc.robot.Constants;

public class CameraVariables {
    public static final int IMAGE_FPS = 30;

    public static class CalculatedVariables {
        public static double[] Velocity = new double[3];
    }

    public static class FrontCam {
        // private static final String discription = "For finding AprilTags and for
        // distance calculation";
        public static final String name = "FrontCamera";
        public static final String ip = "http://10.2.1.17:5801";
        public static final String kFrontCameraNetworkTablesName = "limelight-april";
        
    }

    public static class BackCam {
        // private static final String discription = "For finding AprilTags and going
        // towards them";
        public static final String kName = "BackCamera";
        public static final String ip = "http://10.2.1.43:5801";
        public static final String kBackCameraNetworkTablesName = "limelight-notes";
        public static double horizontal_fov = 62.5;
        public static int CameraWidth = 640;
        public static int CameraHeight = 480;

    }
}