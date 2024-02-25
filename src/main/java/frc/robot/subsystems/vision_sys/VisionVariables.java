package frc.robot.subsystems.vision_sys;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.vision_sys.utils.VisionObject;

import java.util.Optional;

public class VisionVariables {

    public static Optional<DriverStation.Alliance> isBlueAlliance;
    public static class ExportedVariables {
        public static double Distance;
        public static int AngleForShooter;
    }

    public static class FrontCam {
        
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