package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Vision.utils.VisionObject;

public class LimelightUtils {
    public static double GetAngle(double limelightDistance) {
        SmartDashboard.putNumber("Supplied Distance to Shooter", limelightDistance);
        if (!VisionObject.isPresent()) {
            return -34;
        } else if (limelightDistance < 0.5) {
            return -1;
        } else if (limelightDistance < 1.125) { // 1
            return -6;
        } else if (limelightDistance < 1.375) { // 1.25
            return -12;
        } else if (limelightDistance < 1.625) { // 1.5
            return -17;
        } else if (limelightDistance < 1.875) { // 1.75
            return -20;
        } else if(limelightDistance<5){
            return -1 * (ShooterConstants.A * (limelightDistance * limelightDistance) + ShooterConstants.B * limelightDistance + ShooterConstants.C);
        } else{
            return -34;
        }
        //Values measures until 3.75 limelight distance
//        else if (limelightDistance < 2.375) { // 2.25
//            return -25.5;
//        } else if (limelightDistance < 2.625) { // 2.5
//            return -27.5;
//        } else if (limelightDistance < 2.875) { // 2.75
//            return -30;
//        } else if (limelightDistance < 3.125) { // 3
//            return -31.5;
//        } else if (limelightDistance < 3.375) { // 3.25
//            return -32.25;
//        } else if (limelightDistance < 3.625) { // 3.5
//            return -33.5;
//        } else if (limelightDistance < 3.875) { // 3.75
//            return -34;
//        } else if (limelightDistance < 4.125) { // 4
//            return -38;
//        } else if (limelightDistance < 4.375) { // 4.25
//            return -39;
//        } else if (limelightDistance < 4.625) { // 4.5
//            return -42;
//        } else if (limelightDistance < 4.875) { // 4.75
//            return -43;
//        } else if (limelightDistance < 5.125) { // 5
//            return -45;
//        } else { // 5.25
//            return -46;
//        }
        /*else if (limelightDistance < 5.625) { // 5.5
            return -41;
        } else if (limelightDistance < 5.875) { // 5.75
            return -42;
        } else if (limelightDistance < 6.125) { // 6
            return -43;
        } else if (limelightDistance < 6.375) { // 6.25
            return -44;
        } else if (limelightDistance < 6.625) { // 6.5
            return -45;
        } else if (limelightDistance < 6.875) { // 6.75
            return -46;
        } else if (limelightDistance < 7.125) { // 7
            return -47;
        } else if (limelightDistance < 7.375) { // 7.25
            return -48;
        } else if (limelightDistance < 7.625) { // 7.5
            return -49;
        } else if (limelightDistance < 7.875) { // 7.75
            return -49;
        } else { // 8
            return -49.5;
        }*/
        
    }

    public static double GetSpeedTop(double limelightDistance) {
        return -80;
        // Right now there is no change in speed depending on the distance
    }
    
    public static double GetSpeedBottom(double limelightDistance) {
        return -80;
    }
    
    public static double MapDistanceToOffset(double asDouble) {
        return 0;
    }
}