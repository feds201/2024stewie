package frc.robot.subsystems.vision_sys.utils;

public class VisionObject {
    private double x;
    private double y;
    private double area;

    public VisionObject(double x, double y, double area) {
        this.x = x;
        this.y = y;
        this.area = area;
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
}