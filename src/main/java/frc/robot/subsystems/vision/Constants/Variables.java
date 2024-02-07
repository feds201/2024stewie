package frc.robot.subsystems.vision.Constants;

public class Variables {
    public static final int IMAGE_WIDTH = 320;
    public static final int IMAGE_HEIGHT = 240;
    public static final int IMAGE_FPS = 30;
    public static final int IMAGE_PIXEL_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT;
    public static final String frontCam_Name = "FrontCamera";
    public static final String frontCam_EName = "limelight-april";
    public static final String frontCam_IP = "http://10.2.1.17:5801";

    public static final String backCam_Name = "BackCamera";
    public static final String backCam_EName = "limelight-notes";
    public static final String backCam_IP = "http://10.2.1.43:5801";

    public static class ExportedVariables {
        public static double[] Velocity = new double[3];
        
    }

    public static class FrontCam {
        // private static final String discription = "For finding AprilTags and for distance calculation";
        public static final String name = frontCam_Name;
        public static final String ip = frontCam_IP;
        public static int tv;
        public static double tx;
        public static double ty;
        public static double ta;
        public static int tid;
        public static Number CameraMode;

    }

    public static class BackCam {
        // private static final String discription = "For finding AprilTags and going towards them";
        public static final String name = backCam_Name;
        public static final String ip = backCam_IP;
        public static int tv;
        public static double tx;
        public static double ty;
        public static double ta;
        public static int tid;
        public static Number CameraMode;

    }

}