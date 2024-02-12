package frc.robot.subsystems.vision_sys.camera;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.vision_sys.utils.DashBoardManager;
import frc.robot.subsystems.vision_sys.utils.ObjectType;
import frc.robot.subsystems.vision_sys.utils.VisionObject;
import frc.robot.subsystems.vision_sys.vision_sys;

import static frc.robot.subsystems.vision_sys.utils.ObjectType.APRILTAG;

public class BackCamera extends vision_sys {
    public static ObjectType type;
    public static DashBoardManager dashBoardManager;
    public static String nt_key;
    public static NetworkTable table;
    public static VisionObject tag;

    public BackCamera() {
        type = APRILTAG;
        dashBoardManager = new DashBoardManager();
        nt_key = CameraConstants.BackCam.BACK_CAMERA_NETWORK_TABLES_NAME;
        table = NetworkTableInstance.getDefault().getTable(nt_key);
        tag = new VisionObject(0, 0, 0, type);
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean CheckTarget() {
        return false;
    }

    @Override
    public Translation2d GetTarget(VisionObject note) {
        return null;
    }

}