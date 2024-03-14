package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightUtils {
    public static class SpeedAngle {
        public double speed;
        public double angle;

        public SpeedAngle(double s, double a) {
            this.speed = s;
            this.angle = a;
        }

        public SpeedAngle() {
            this.speed = 0;
            this.angle = 0;
        }
    }

    public static SpeedAngle GetSpeedAngle(double limelightDistance) {
        SpeedAngle speedAngle = new SpeedAngle();
        speedAngle.angle = getAngle(limelightDistance);
        speedAngle.speed = getSpeed(limelightDistance);
        return speedAngle;
    }

    private static double getAngle(double limelightDistance) {
        SmartDashboard.putNumber("Supplied Distance to Shooter", limelightDistance);
        if (limelightDistance < 1) {
            return -5;
        } else if (limelightDistance < 1.125) { // 1
            return -5;
        } else if (limelightDistance < 1.375) { // 1.25
            return -15;
        } else if (limelightDistance < 1.625) { // 1.5
            return -16;
        } else if (limelightDistance < 1.875) { // 1.75
            return -17;
        } else if (limelightDistance < 2.125) { // 2
            return -18;
        } else if (limelightDistance < 2.375) { // 2.25
            return -19;
        } else if (limelightDistance < 2.625) { // 2.5
            return -23;
        } else if (limelightDistance < 2.875) { // 2.75
            return -25;
        } else if (limelightDistance < 3.125) { // 3
            return -27;
        } else if (limelightDistance < 3.375) { // 3.25
            return -29;
        } else if (limelightDistance < 3.625) { // 3.5
            return -31;
        } else if (limelightDistance < 3.875) { // 3.75
            return -33;
        } else if (limelightDistance < 4.125) { // 4
            return -35;
        } else if (limelightDistance < 4.375) { // 4.25
            return -29;
        } else if (limelightDistance < 4.625) { // 4.5
            return -30;
        } else if (limelightDistance < 4.875) { // 4.75
            return -29;
        } else if (limelightDistance < 5.125) { // 5
            return -30;
        } else if (limelightDistance < 5.375) { // 5.25
            return -31;
        } else if (limelightDistance < 5.625) { // 5.5
            return -32;
        } else if (limelightDistance < 5.875) { // 5.75
            return -33;
        } else if (limelightDistance < 6.125) { // 6
            return -34;
        } else if (limelightDistance < 6.375) { // 6.25
            return -35;
        } else if (limelightDistance < 6.625) { // 6.5
            return -36;
        } else if (limelightDistance < 6.875) { // 6.75
            return -57;
        } else if (limelightDistance < 7.125) { // 7
            return -57.5;
        } else if (limelightDistance < 7.375) { // 7.25
            return -58;
        } else if (limelightDistance < 7.625) { // 7.5
            return -59;
        } else if (limelightDistance < 7.875) { // 7.75
            return -59;
        } else { // 8
            return -59.5;
        }
    }

    private static double getSpeed(double limelightDistance) {
        if(limelightDistance < 2) {
            return -60;
        } else {
            return -80;
        }
    }
}