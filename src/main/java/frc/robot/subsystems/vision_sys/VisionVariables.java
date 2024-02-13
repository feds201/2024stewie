package frc.robot.subsystems.vision_sys;

import frc.robot.subsystems.vision_sys.utils.VisionObject;

public class VisionVariables {
    public static class ExportedVariables {
        public static Double Distance;
        public static int AngleForShooter;


    }

    public static class FrontCam {
        private static final String description = "For finding Notes and going towards them";
        public static int tv;
        public static VisionObject target;
        public static Number CameraMode;
        public static class RobotTransformation {
            public static double x;
            public static double y;
            public static double rotation;
        }
        

    }

    public static class BackCam {
        private static final String description = "For finding AprilTags and for distance calculation";
        public static int tv;
        public static VisionObject target;
        public static int tid;
        public static Number CameraMode;
        public static class RobotTransformation {
            public static double x;
            public static double y;
            public static double rotation;
        }

    }

}