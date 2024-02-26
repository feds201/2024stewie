package frc.robot.utils;

public class LLDistanceToShooterAngle {
    public static double LLDistanceToShooterAngle(double limelightDistance) {
        if(limelightDistance < 1) {
            return 20;
        } else if (limelightDistance < 1.125) {
            return 20;
        } else if (limelightDistance < 1.375) {
            return 20;
        } else if (limelightDistance < 1.625) {
            return 23;
        } else if (limelightDistance < 1.875) {
            return 25;
        } else if (limelightDistance < 2.125) {
            return 28;
        } else if (limelightDistance < 2.375) {
            return 30;
        } else if (limelightDistance < 2.625) {
            return 34;
        } else if (limelightDistance < 2.875) {
            return 36;
        } else {
            return 37;
        }
    }
}