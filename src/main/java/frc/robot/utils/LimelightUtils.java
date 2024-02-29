package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightUtils {
    public static double GetShooterAngle(double limelightDistance) {
        SmartDashboard.putNumber("Supplied Distance to Shooter", limelightDistance);
        if (limelightDistance < 1) {
            return -22;
        } else if (limelightDistance < 1.125) { // 1
            return -23;
        } else if (limelightDistance < 1.375) { // 1.25
            return -23;
        } else if (limelightDistance < 1.625) { // 1.5
            return -26;
        } else if (limelightDistance < 1.875) { // 1.75
            return -27;
        } else if (limelightDistance < 2.125) { // 2
            return -30;
        } else if (limelightDistance < 2.375) { // 2.25
            return -32;
        } else if (limelightDistance < 2.625) { // 2.5
            return -36;
        } else if (limelightDistance < 2.875) { // 2.75
            return -38;
        } else if (limelightDistance < 3.125) { // 3
            return -39;
        } else if (limelightDistance < 3.375) { // 3.25
            return -41;
        } else if (limelightDistance < 3.625) { // 3.5
            return -42;
        } else if (limelightDistance < 3.875) { // 3.75
            return -44;
        } else if (limelightDistance < 4.125) { // 4
            return -45;
        } else if (limelightDistance < 4.375) { // 4.25
            return -46;
        } else if (limelightDistance < 4.625) { // 4.5
            return -47;
        } else if (limelightDistance < 4.875) { // 4.75
            return -48;
        } else if (limelightDistance < 5.125) { // 5
            return -51;
        } else if (limelightDistance < 5.375) { // 5.25
            return -52.5;
        } else if (limelightDistance < 5.625) { // 5.5
            return -53;
        } else if (limelightDistance < 5.875) { // 5.75
            return -54;
        } else if (limelightDistance < 6.125) { // 6
            return -55;
        } else if (limelightDistance < 6.375) { // 6.25
            return -56;
        } else if (limelightDistance < 6.625) { // 6.5
            return -56;
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
        } else {                                // 8
            return -59.5;
        }
    }
}