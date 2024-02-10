package frc.robot.subsystems.vision;

public class VisionVariables {
    public static class ExportedVariables {
        public static double[] Velocity = new double[3];
        public static Double Distance;
        public static int AngleForShooter;
    }

    public static class FrontCam {
        // private static final String discription = "For finding Notes and going towards them";
        public static int tv;
        public static double tx;
        public static double ty;
        public static double ta;
        public static Number CameraMode;
        

    }

    public static class BackCam {
        // private static final String discription = "For finding AprilTags and for distance calculation";
        public static int tv;
        public static double tx;
        public static double ty;
        public static double ta;
        public static int tid;
        public static Number CameraMode;

    }

}