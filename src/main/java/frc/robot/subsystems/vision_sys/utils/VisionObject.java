package frc.robot.subsystems.vision_sys.utils;

import frc.robot.constants.CameraConstants;

public class VisionObject {
    private double x;
    private double y;
    private double area;
    private ObjectType type;

    public VisionObject(double x, double y, double area, ObjectType type) {
        this.x = x;
        this.y = y;
        this.area = area;
        this.type = type;
    }

    public double getX() {
        return x;
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

    public ObjectType getType() {
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
}