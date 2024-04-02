package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Vision.utils.VisionObject;

public class LimelightUtils {
    private static double a = 2; //aiming variance (this might have to be a table for better accuracy)
    private static double b = -1;

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

    public static double MapDistanceToOffset(double limelightDistance){
        if(limelightDistance < 1) {
            return 0;
        } else if (limelightDistance < 2) {
            return -3;
        } else if (limelightDistance < 3) {
            return -6;
        } else if(limelightDistance < 4){
            return -7;
        }else if(limelightDistance < 5) {    
            return -7;
        } else if (limelightDistance < 6) {
            return -8;
        } else {
            return -10;
        }
    }

//    public static double getTrigAlignAngle(double limelightTagDistanceCenter, double limelightTagDistanceRight) {
//        boolean whichWay = true; //replace with some boolean that you pass in as a parameter that tells us which way the limelight wants the robot to turn to shoot
//        double y = limelightTagDistanceRight; //Reading for the distance to the center AprilTag on speaker
//        double x = limelightTagDistanceCenter; //Reading for the distane to the right AprilTag on speaker
//        double z = 15.75; //Constant: Distance of AprilTag to AprilTag
//        b = Math.sqrt(Math.pow(x, 2) + (Math.pow(z, 2) / Math.pow(a, 2)) + ((Math.pow(y, 2) - Math.pow(x, 2) - Math.pow(z, 2)) / a ));
//        double totalAngle = Math.asin( (z * Math.sqrt( 1 - Math.pow( ((Math.pow(y, 2) - Math.pow(x, 2) - Math.pow(z, 2)) / -2 * a * x ), 2) )) / y); //The total angle that the limelight can see between the 2 AprilTags
//        double rotateAngleRight = Math.asin( (Math.sqrt(1 - Math.pow(( (Math.pow(y, 2) - Math.pow(x, 2) - Math.pow(z, 2)) / (-2 * x * z) ), 2))) / (x * Math.sqrt( ((-1 * (Math.pow(y, 2))) + Math.pow(x, 2) + Math.pow(z, 2) / a) ))); //The angle for the robot to correct to the right based on where we set the variance
//        double rotateAngleLeft = totalAngle - rotateAngleRight; //The angle for the robot to correct to the left based on where we set the variance
//
//        SmartDashboard.putNumber("TRIG: Total Angle", totalAngle);
//        SmartDashboard.putNumber("TRIG: Correction Angle Left", rotateAngleLeft);
//        SmartDashboard.putNumber("TRIG: Correction Angle Right", rotateAngleRight);
//        // if(whichWay) { //if the robot needs to turn right to aim at the variance
//        //     return rotateAngleRight;
//        // } else if (!whichWay) { //if the robot needs to turn left to aim at the variance
//        //     return rotateAngleLeft;
//        // }
//
//        return rotateAngleLeft;
//    }

    public static double getArcCoordinatesY(double xCoord) {
        //(x-a)^2 + y^2 = b^2
        //y = sqrt(b^2 - (x-a)^2)
        double y = Math.sqrt(Math.pow(b, 2) - Math.pow(xCoord - (15.75/a), 2));
        SmartDashboard.putNumber("TRIG: Arc calculation X", xCoord);
        SmartDashboard.putNumber("TRIG: Arc calculation Y", y);
        return y;
    }

    private static double getAngle(double limelightDistance) {
        SmartDashboard.putNumber("Supplied Distance to Shooter", limelightDistance);
        if (!VisionObject.isPresent()){
            return -33;
        } else if (limelightDistance < 0.5) {
            return -1;
        } else if (limelightDistance < 1.125) { // 1
            return -6;
        } else if (limelightDistance < 1.375) { // 1.25
            return -15;
        } else if (limelightDistance < 1.625) { // 1.5
            return -16;
        } else if (limelightDistance < 1.875) { // 1.75
            return -17;
        } else if (limelightDistance < 2.125) { // 2
            return -19.5;
        } else if (limelightDistance < 2.375) { // 2.25
            return -20;
        } else if (limelightDistance < 2.625) { // 2.5
            return -24.5;
        } else if (limelightDistance < 2.875) { // 2.75
            return -29;
        } else if (limelightDistance < 3.125) { // 3
            return -27;
        } else if (limelightDistance < 3.375) { // 3.25
            return -29;
        } else if (limelightDistance < 3.625) { // 3.5
            return -31;
        } else if (limelightDistance < 3.875) { // 3.75
            return -33;
        } else if (limelightDistance < 4.125) { // 4
            return -33;
        } else if (limelightDistance < 4.375) { // 4.25
            return -31;
        } else if (limelightDistance < 4.625) { // 4.5
            return -34.5;
        } else if (limelightDistance < 4.875) { // 4.75
            return -29;
        } else if (limelightDistance < 5.125) { // 5
            return -34;
        } else if (limelightDistance < 5.375) { // 5.25
            return -32.5;
        } else if (limelightDistance < 5.625) { // 5.5
            return -32;
        } else if (limelightDistance < 5.875) { // 5.75
            return -34.2;
        } else if (limelightDistance < 6.125) { // 6
            return -34;
        } else if (limelightDistance < 6.375) { // 6.25
            return -35;
        } else if (limelightDistance < 6.625) { // 6.5
            return -36;
        } else if (limelightDistance < 6.875) { // 6.75
            return -37;
        } else if (limelightDistance < 7.125) { // 7
            return -37.5;
        } else if (limelightDistance < 7.375) { // 7.25
            return -38;
        } else if (limelightDistance < 7.625) { // 7.5
            return -39;
        } else if (limelightDistance < 7.875) { // 7.75
            return -39;
        } else { // 8
            return -39.5;
        }
    }

    private static double getSpeed(double limelightDistance) {

        if(limelightDistance < 2) {
            return -80;
        } else {
            return -80;
        }
    }
}