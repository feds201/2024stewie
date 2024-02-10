package frc.robot.subsystems.vision.Components;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.vision.VisionVariables;
// AprilTag Camera
public class BackCamera extends Camera{
    public static String nt_key = CameraConstants.BackCam.kBackCameraNetworkTablesName;
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable(nt_key);
    @Override
    public void periodic() {
        
        VisionVariables.BackCam.tx = table.getEntry("tx").getNumber(0).doubleValue();
        VisionVariables.BackCam.tv = (int) table.getEntry("tv").getNumber(0).doubleValue();
        VisionVariables.BackCam.ty = table.getEntry("ty").getNumber(0).doubleValue();
        VisionVariables.BackCam.ta = table.getEntry("ta").getNumber(0).doubleValue();
        VisionVariables.BackCam.tid = (int) table.getEntry("tid").getNumber(0).doubleValue();

        VisionVariables.BackCam.CameraMode = table.getEntry("camMode").getNumber(0);
        SmartDashboard.putNumber("Xaxis", VisionVariables.BackCam.tx);
        SmartDashboard.putNumber("Yaxis", VisionVariables.BackCam.ty);
        SmartDashboard.putNumber("Target ID", VisionVariables.BackCam.tid);
        SmartDashboard.putBoolean("Target Locked", checkTag());
        SmartDashboard.putNumber("Angle of AprilTAG", getAngle());
    }

    public double getAngle() {
        // using formulas from here: https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/retro-theory

        double ny = (1/(CameraConstants.FrontCam.CameraHeight/2)) * (VisionVariables.FrontCam.tx - (CameraConstants.FrontCam.CameraWidth/2-0.5));
        double vpw = 2.0 * Math.tan(CameraConstants.BackCam.horizontal_fov / 2);
        double y = vpw / 2 * ny;
        double ay = Math.atan2(1, y);
        return Math.toDegrees(ay);
    }
    public boolean checkTag() {
        return VisionVariables.BackCam.tv != 0;
    }




    
}
