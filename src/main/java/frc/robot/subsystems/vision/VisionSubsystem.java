package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Components.BackCamera;
import frc.robot.subsystems.vision.Components.FrontCamera;

public class VisionSubsystem extends SubsystemBase {
    private final FrontCamera frontCameraData;
    private final BackCamera backCameraData;

    public VisionSubsystem() {
        this.frontCameraData = new FrontCamera();
        this.backCameraData = new BackCamera();
    }

    public void periodic() {
        frontCameraProperties();
        backCameraProperties();
    }

    public void frontCameraProperties() {
        String moduleName = this.frontCameraData.getModuleName();
        // other properties...


        SmartDashboard.putString("moduleName", moduleName);
        // other dashboard updates...
    }

    public void backCameraProperties() {
        String moduleName = this.backCameraData.getModuleName();
        // other properties...


        SmartDashboard.putString("moduleName", moduleName);
        // other dashboard updates...
    }
}