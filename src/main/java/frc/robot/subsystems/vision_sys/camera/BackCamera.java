package frc.robot.subsystems.vision_sys.camera;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.vision_sys.VisionVariables;
import frc.robot.subsystems.vision_sys.utils.DashBoardManager;
import frc.robot.subsystems.vision_sys.utils.ObjectType;
import frc.robot.subsystems.vision_sys.utils.VisionObject;
import frc.robot.subsystems.vision_sys.vision_sys;

public class BackCamera extends vision_sys {
    public static DashBoardManager dashBoardManager;
    public static String nt_key;
    public static NetworkTable table;
    public static VisionObject tag;

    public BackCamera() {
        dashBoardManager = new DashBoardManager();
        nt_key = CameraConstants.BackCam.BACK_CAMERA_NETWORK_TABLES_NAME;
        table = NetworkTableInstance.getDefault().getTable(nt_key);
        tag = new VisionObject(0, 0, 0, ObjectType.APRILTAG);
    }

    @Override
    public void periodic() {
        tag.update(
                table.getEntry("tx").getNumber(0).doubleValue(),
                table.getEntry("ty").getNumber(0).doubleValue(),
                table.getEntry("ta").getNumber(0).doubleValue()
        );
        VisionVariables.BackCam.tv = table.getEntry("tv").getNumber(0).intValue();
        dashBoardManager.DashBoard("BackCamera", tag.getX(), tag.getY(), CheckTarget(), tag.getAngle());

        SmartDashboard.putNumber("Distance", getDistance(tag));
        //
//        if (CheckTarget()) {
//            VisionVariables.BackCam.RobotTransformation.rotation = tag.getYaw();
//            VisionVariables.BackCam.RobotTransformation.x = getDistance(tag);
//
//        }

    }

    @Override
    public boolean CheckTarget() {
        return VisionVariables.BackCam.tv != 0;
    }

    public  double getDistance(VisionObject tag){
        return tag.getDistance();
    }

    @Override
    public Translation2d GetTarget(VisionObject note) {
        return null;
    }

}