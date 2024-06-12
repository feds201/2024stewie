/*
 * *****************************************************************************
 *  * Copyright (c) 2024 FEDS 201. All rights reserved.
 *  *
 *  * This codebase is the property of FEDS 201 Robotics Team.
 *  * Unauthorized copying, reproduction, or distribution of this code, or any
 *  * portion thereof, is strictly prohibited.
 *  *
 *  * This code is provided "as is" and without any express or implied warranties,
 *  * including, without limitation, the implied warranties of merchantability
 *  * and fitness for a particular purpose.
 *  *
 *  * For inquiries or permissions regarding the use of this code, please contact
 *  * feds201@gmail.com
 *  ****************************************************************************
 *
 */

package frc.robot.subsystems.Vision.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.Vision.VisionVariables;
import frc.robot.subsystems.Vision.utils.ObjectType;

public class VisionObject {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable(CameraConstants.BackCam.BACK_CAMERA_NETWORK_TABLES_NAME);
    private double x;
    private double y;
    private double area;
    private ObjectType type;


    public VisionObject(double x, double y, double area, frc.robot.subsystems.Vision.utils.ObjectType type) {
        this.x = x;
        this.y = y;
        this.area = area;
        this.type = type;
    }


    public double getX() {
        return x;
    }

    public static boolean isPresent(){
		return table.getEntry("tv").getDouble(0) == 1.0;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getArea() {
        return area;
    }

    public void setArea(double area) {
        this.area = area;
    }

    public void update(double x, double y, double area) {
        this.x = x;
        this.y = y;
        this.area = area;
    }

    public frc.robot.subsystems.Vision.utils.ObjectType getType() {
        return type;
    }

    public void setType(ObjectType type) {
        this.type = type;
    }

    public double[] getAngle() {
        switch (type) {
            case NOTE:
                double[] angle = new double[2];
                double nx = ((double) 1 / ((double) CameraConstants.FrontCam.CameraWidth / 2)) * (x - ((double) CameraConstants.FrontCam.CameraWidth / 2 - 0.5));
                double ny = ((double) 1 / ((double) CameraConstants.FrontCam.CameraHeight / 2)) * (y - ((double) CameraConstants.FrontCam.CameraHeight / 2 - 0.5));
                double vpw = 2.0 * Math.tan(CameraConstants.FrontCam.horizontal_fov / 2);
                double vph = 2.0 * Math.tan(CameraConstants.FrontCam.vertical_fov / 2);
                double x = vpw / 2 * nx;
                double y = vph / 2 * ny;
                double ax = Math.atan2(1, x);
                double ay = Math.atan2(1, y);
                angle[0] = Math.toDegrees(ax);
                angle[1] = Math.toDegrees(ay);
                return angle;
            case APRILTAG:
                double[] angle1 = new double[2];
                double nx1 = ((double) 1 / ((double) CameraConstants.BackCam.CameraWidth / 2)) * (this.x - ((double) CameraConstants.BackCam.CameraWidth / 2 - 0.5));
                double ny1 = ((double) 1 / ((double) CameraConstants.BackCam.CameraHeight / 2)) * (this.y - ((double) CameraConstants.BackCam.CameraHeight / 2 - 0.5));
                double vpw1 = 2.0 * Math.tan(CameraConstants.BackCam.horizontal_fov / 2);
                double vph1 = 2.0 * Math.tan(CameraConstants.BackCam.vertical_fov / 2);
                double x1 = vpw1 / 2 * nx1;
                double y1 = vph1 / 2 * ny1;
                double ax1 = Math.atan2(1, x1);
                double ay1 = Math.atan2(1, y1);
                angle1[0] = Math.toDegrees(ax1);
                angle1[1] = Math.toDegrees(ay1);
                return angle1;
            default:
                throw new IllegalArgumentException("Invalid type");
        }
    }

    public double getYaw() {
        return switch (type) {
            case NOTE -> getAngle()[0];
            case APRILTAG -> getAngle()[0];
            default -> throw new IllegalArgumentException("Invalid type");
        };
    }

    public double getPitch() {
        return switch (type) {
            case NOTE -> getAngle()[1];
            case APRILTAG -> getAngle()[1];
            default -> throw new IllegalArgumentException("Invalid type");
        };
    }

    public double getDistance() {
        switch (type) {
            case NOTE:
                throw new IllegalArgumentException("Finish the code first before doing something stupid");
            case APRILTAG:
                double targetOffsetAngle_Vertical = y;
                double limelightMountAngleDegrees = 30;
                double limelightLensHeightMetres = 0.285;
                double goalHeightMetres = 1.4;
                double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
                return (goalHeightMetres - limelightLensHeightMetres) / Math.tan(angleToGoalRadians);
            default:
                throw new IllegalStateException("Unexpected value: " + type);
        }
    }

    public Pose3d getBotPose(){
        return LimelightHelpers.getBotPose3d(CameraConstants.BackCam.BACK_CAMERA_NETWORK_TABLES_NAME);
    }

}