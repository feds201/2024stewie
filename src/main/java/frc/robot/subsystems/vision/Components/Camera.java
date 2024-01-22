package frc.robot.subsystems.vision.Components;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
    public final NetworkTableInstance table = NetworkTableInstance.getDefault();

    // Properties
    public String moduleName;
    public double pipelineName;
    public double ledMode;
    public double tx;
    double ty;
    double ta;
    boolean tv;

    public Camera() {
    }

    public void periodic() {
        // Implementation here
    }

    // Getter methods
    public String getModuleName() {
        return moduleName;
    }

    public double getPipelineName() {
        return pipelineName;
    }

    public double getLedMode() {
        return ledMode;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTa() {
        return ta;
    }

    public boolean isTv() {
        return tv;
    }
}