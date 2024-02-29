package frc.robot.constants;

public class DIOConstants {
    public static class Shooter {
        public static final int kShooterRotateEncoder = 1; // Determine which is which!
    }

    public static class Intake {
        public static final int kIntakeRotateEncoder = 2;
    }

    public static class Arm {
        public static final int kArmRotateEncoder = 0;
    }

    public static class SensorConstants {
        public static final int shooterBreakBeamReceiverPort = 8;
        public static final int shooterBreakBeamTransmitterPort = 9;
        public static final int intakeBreakBeamReceiverPort = 6;
        public static final int intakeBreakBeamTransmitterPort = 7;
    }
}
