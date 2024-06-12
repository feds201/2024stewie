package frc.robot.subsystems.Vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Vision.utils.VisionObject;

public class VisionVariables {

    public static class ExportedVariables {

        public static double Distance;
    }
    public Field2d RobotField;
    public Transform3d RobotTransformation;

    public static class FrontCam {
        public static int tv;
        public static VisionObject target;
        public static Number CameraMode;
        public static double distance;
		    public static int LEDMode;

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
        public static int LEDMode;

        public static class RobotTransformation {
            public static double x;
            public static double y;
            public static double rotation;
        }
    }
}