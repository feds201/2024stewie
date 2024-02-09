package frc.robot.subsystems.vision_systems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CameraVariables;
import java.lang.Math;

public class BackCamera extends Camera {
    public static String nt_key = CameraVariables.BackCam.kBackCameraNetworkTablesName;
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable(nt_key);

    public BackCamera() {
    }

    public static class Values {

    }

    public void periodic() {
        CameraVariables.BackCam.tx = table.getEntry("tx").getNumber(0).doubleValue();
        CameraVariables.BackCam.tv = (int) table.getEntry("tv").getNumber(0).doubleValue();
        CameraVariables.BackCam.ty = table.getEntry("ty").getNumber(0).doubleValue();
        CameraVariables.BackCam.ta = table.getEntry("ta").getNumber(0).doubleValue();
        CameraVariables.BackCam.CameraMode = table.getEntry("camMode").getNumber(0);
        SmartDashboard.putNumber("Xaxis", CameraVariables.BackCam.tx);
        SmartDashboard.putNumber("Yaxis", CameraVariables.BackCam.ty);
        SmartDashboard.putNumber("Target ID", CameraVariables.BackCam.tid);
        SmartDashboard.putBoolean("Target Locked", true);
        
        SmartDashboard.putNumber("Angle", getAngle());
    }

    public int getAngle() {
        // using formulas from here: https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/retro-theory

        double nx = (1/(CameraVariables.BackCam.CameraWidth/2)) * (CameraVariables.BackCam.tx - (CameraVariables.BackCam.CameraWidth/2-0.5));
        double vpw = 2.0 * Math.tan(CameraVariables.BackCam.horizontal_fov / 2);
        double x = vpw / 2 * nx;
        double ax = Math.atan2(1, x);
        // SmartDashboard.putNumber("Angle", ax);
        return (int) Math.toDegrees(ax);
    }

    public boolean checkNote() {
        return CameraVariables.BackCam.tv != 0;
    }

    public Translation2d getTargetLocation() {
        return new Translation2d(CameraVariables.BackCam.tx, CameraVariables.BackCam.ty);
    }

}
