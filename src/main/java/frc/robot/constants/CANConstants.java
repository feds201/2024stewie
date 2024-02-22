package frc.robot.constants;

public class CANConstants {
    public static class Swerve {
        public static final int kPigeon = 1;            

        public static final int kDriveFrontLeft     = 21; // Falcon
        public static final int kTurnFrontLeft      = 22; // Falcon
        public static final int kCANCoderFrontLeft  = 23; // Falcon

        public static final int kDriveFrontRight    = 11; // Falcon
        public static final int kTurnFrontRight     = 12; // Falcon
        public static final int kCANCoderFrontRight = 13; // Falcon

        public static final int kDriveBackLeft      = 41; // Falcon
        public static final int kTurnBackLeft       = 42; // Falcon
        public static final int kCANCoderBackLeft   = 43; // Falcon

        public static final int kDriveBackRight     = 31; // Falcon
        public static final int kTurnBackRight      = 32; // Falcon
        public static final int kCANCoderBackRight  = 33; // Falcon
    }

    public static class Shooter {
        public static final int kShooterTop    = 3;       // Falcon
        public static final int kShooterBottom = 4;       // Falcon
        public static final int kShooterPivot  = 5;       // Kraken
    }

    public static class Arm {
        public static final int kArm = 6;                 // Kraken
    }

    public static class Intake {
        public static final int kIntakeWheels = 50;       // Rev
        public static final int kIntakeWrist = 55;        // Rev
    }

    public static class Climber {
        public static final int kClimberRightMain = 51;   // Rev 
        public static final int kClimberLeftFollower = 52;// Rev
    }
}
