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
        frontCameraData.Periodic();
        backCameraData.Periodic();
        SmartDashboard.putNumber("Front Camera Pipeline", frontCameraData.getPipelineName());
        SmartDashboard.putNumber("Front Camera LED Mode", frontCameraData.getLedMode());
        SmartDashboard.putNumber("Front Camera tx", frontCameraData.getTx());
        SmartDashboard.putNumber("Front Camera ty", frontCameraData.getTy());
        SmartDashboard.putNumber("Front Camera ta", frontCameraData.getTa());
        SmartDashboard.putBoolean("Front Camera tv", frontCameraData.isTv());
        SmartDashboard.putNumber("Back Camera Pipeline", backCameraData.getPipelineName());
        SmartDashboard.putNumber("Back Camera LED Mode", backCameraData.getLedMode());
        SmartDashboard.putNumber("Back Camera tx", backCameraData.getTx());
        SmartDashboard.putNumber("Back Camera ty", backCameraData.getTy());
        SmartDashboard.putNumber("Back Camera ta", backCameraData.getTa());
        SmartDashboard.putBoolean("Back Camera tv", backCameraData.isTv());
    }

    public double getDistance(double target_ta) {
        if (target_ta <= 0) {
            throw new IllegalArgumentException("Target area must be greater than 0 inches");
        }
        return Math.sqrt(1 / target_ta);
    }

    /**
     * Calculates the distance based on the difference in heights and the sum of two angles.
     *
     * @param h1 The height at the first point.
     * @param h2 The height at the second point. Must be greater than h1.
     * @param a1 The angle at the first point in degrees.
     * @param a2 The angle at the second point in degrees.
     * @return The calculated distance.
     * @throws IllegalArgumentException If h2 is less than or equal to h1.
     */
    public double getDistance_v2(double h1, double h2, double a1, double a2) {
        if (h2 <= h1) {
            throw new IllegalArgumentException("h2 must be greater than h1");
        }
        double radianA1 = Math.toRadians(a1);
        double radianA2 = Math.toRadians(a2);
        return (h2 - h1) / Math.tan(radianA1 + radianA2);
    }

}