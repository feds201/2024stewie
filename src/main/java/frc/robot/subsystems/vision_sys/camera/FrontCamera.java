package frc.robot.subsystems.vision_sys.camera;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.vision_sys.DashBoardManager;
import frc.robot.subsystems.vision_sys.VisionVariables;
import frc.robot.subsystems.vision_sys.utils.VisionObject;
import frc.robot.subsystems.vision_sys.vision_sys;

public class FrontCamera extends vision_sys {
    public static DashBoardManager dashBoardManager = new DashBoardManager();
    public static String nt_key = CameraConstants.FrontCam.kFrontCameraNetworkTablesName;
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable(nt_key);
    public static VisionObject note = new VisionObject(0, 0, 0);

    @Override
    public void periodic() {
        note.update(
                table.getEntry("tx").getNumber(0).doubleValue(),
                table.getEntry("ty").getNumber(0).doubleValue(),
                table.getEntry("ta").getNumber(0).doubleValue()
        );
        dashBoardManager.DashBoard("FrontCamera", note.getX(), note.getY(), CheckTarget(), getAngle(note));
        VisionVariables.FrontCam.tv = (int) table.getEntry("tv").getNumber(0).doubleValue();
        VisionVariables.FrontCam.CameraMode = table.getEntry("camMode").getNumber(0);
    }

    public double getAngle(VisionObject note) {
        double nx = ((double) 1 / ((double) CameraConstants.FrontCam.CameraWidth / 2)) * (note.getX() - ((double) CameraConstants.FrontCam.CameraWidth / 2 - 0.5));
        double vpw = 2.0 * Math.tan(CameraConstants.FrontCam.horizontal_fov / 2);
        double x = vpw / 2 * nx;
        double ax = Math.atan2(1, x);
        return Math.toDegrees(ax);
    }
    public double[] getVelocity(Translation2d noteCords) {
        double deadband = 7.0;
        double x = noteCords.getX();
        double[] velocity = new double[3];

        if (Math.abs(x) < deadband) {
            velocity[0] = 0;
            velocity[1] = 0;
            velocity[2] = 0;
        } else if (x > 0) {
            velocity[0] = 0.5 * CameraConstants.MaxSpeed;
            velocity[1] = 0.5 * CameraConstants.MaxSpeed;
            velocity[2] = 0.1 * CameraConstants.MaxAngularRate;
        } else {
            velocity[0] = -0.5 * CameraConstants.MaxSpeed;
            velocity[1] = -0.5 * CameraConstants.MaxSpeed;
            velocity[2] = -0.1 * CameraConstants.MaxAngularRate;
        }
        SmartDashboard.putNumber("Velocity X", velocity[0]);
        SmartDashboard.putNumber("Velocity Y", velocity[1]);
        SmartDashboard.putNumber("Rotational", velocity[2]);

        return velocity;
    }

    @Override
    public boolean CheckTarget() {
        return VisionVariables.FrontCam.tv != 0;
    }


    public Translation2d GetTarget(VisionObject note) {
        return new Translation2d(note.getX(), note.getY());
    }


}