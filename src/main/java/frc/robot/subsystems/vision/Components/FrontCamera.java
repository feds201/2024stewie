package frc.robot.subsystems.vision.Components;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.subsystems.vision.VisionVariables;
// Note Camera
public class FrontCamera extends Camera{
    
    public static String nt_key = CameraConstants.FrontCam.kFrontCameraNetworkTablesName;
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable(nt_key);

    @Override
    public void periodic() {
        VisionVariables.FrontCam.tx = table.getEntry("tx").getNumber(0).doubleValue();
        VisionVariables.FrontCam.tv = (int) table.getEntry("tv").getNumber(0).doubleValue();
        VisionVariables.FrontCam.ty = table.getEntry("ty").getNumber(0).doubleValue();
        VisionVariables.FrontCam.ta = table.getEntry("ta").getNumber(0).doubleValue();
        VisionVariables.FrontCam.CameraMode = table.getEntry("camMode").getNumber(0);
        SmartDashboard.putNumber("Xaxis", VisionVariables.FrontCam.tx);
        SmartDashboard.putNumber("Yaxis", VisionVariables.FrontCam.ty);
        SmartDashboard.putBoolean("Target Locked", true);
        
        SmartDashboard.putNumber("Angle", getAngle());
    }
    public double getAngle() {
        // using formulas from here: https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/retro-theory

        double nx = (1/(CameraConstants.FrontCam.CameraWidth/2)) * (VisionVariables.FrontCam.tx - (CameraConstants.FrontCam.CameraWidth/2-0.5));
        double vpw = 2.0 * Math.tan(CameraConstants.FrontCam.horizontal_fov / 2);
        double x = vpw / 2 * nx;
        double ax = Math.atan2(1, x);
        return Math.toDegrees(ax);
    }

    public boolean checkNote() {
        return VisionVariables.FrontCam.tv != 0;
    }

    public double[] getVelocity(Translation2d noteCords) {
        double deadband = 7.0; // Adjust the deadband value as needed
        double x = noteCords.getX();
        double[] velocity = new double[3];

        if (Math.abs(x) < deadband) {
            velocity[0] = 0;
            velocity[1] = 0;
            velocity[2] = 0;
        } else if (x > 0) {
            velocity[0] = 0.5 *   CameraConstants.MaxSpeed;
            velocity[1] = 0.5 *   CameraConstants.MaxSpeed;
            velocity[2] = 0.1 *   CameraConstants.MaxAngularRate;
        } else {
            velocity[0] = -0.5 *  CameraConstants.MaxSpeed;
            velocity[1] = -0.5 *   CameraConstants.MaxSpeed;
            velocity[2] = -0.1 *   CameraConstants.MaxAngularRate;
        }
            SmartDashboard.putNumber("Velocity X", velocity[0]);
            SmartDashboard.putNumber("Velocity Y", velocity[1]);
            SmartDashboard.putNumber("Rotational", velocity[2]);

        return velocity;
    }

    public Translation2d getNoteTargetLocation() {
        double x = VisionVariables.FrontCam.tx;
        double y = VisionVariables.FrontCam.ty;
        return new Translation2d(x, y);
    }
    
}
