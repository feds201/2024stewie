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

import java.util.Random;

public class FrontCamera extends vision_sys {
    public static DashBoardManager dashBoardManager;
    public static String nt_key;
    public static NetworkTable table;
    public static VisionObject note;
    public Random random = new Random();

    public FrontCamera() {
        dashBoardManager = new DashBoardManager();
        nt_key = CameraConstants.FrontCam.FRONT_CAMERA_NETWORK_TABLES_NAME;
        table = NetworkTableInstance.getDefault().getTable(nt_key);
        note = new VisionObject(0, 0, 0, ObjectType.NOTE);

    }
    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        note.update(
                random.nextDouble() * 100,
                random.nextDouble() * 100,
                random.nextDouble() * 360
        );
        Periodic();
    }
    @Override
    public void periodic() {
        note.update(
                table.getEntry("tx").getNumber(0).doubleValue(),
                table.getEntry("ty").getNumber(0).doubleValue(),
                table.getEntry("ta").getNumber(0).doubleValue()
        );
        Periodic();

    }
    @Override
    public boolean CheckTarget() {
//        return VisionVariables.FrontCam.tv != 0;
        return true;
    }
    private void Periodic() {
        dashBoardManager.DashBoard(
                "FrontCamera",
                note.getX(),
                note.getY(),
                CheckTarget(),
                note.getAngle()
        );
       VisionVariables.FrontCam.distance = note.getDistance();
       SmartDashboard.putNumber("Note-Distance",note.getDistance());

        VisionVariables.FrontCam.tv = (int) table.getEntry("tv").getNumber(0).doubleValue();
        VisionVariables.FrontCam.CameraMode = table.getEntry("camMode").getNumber(0);

        if (CheckTarget()) {
            VisionVariables.FrontCam.RobotTransformation.rotation = note.getYaw();
        }
    }
    public Translation2d GetTarget(VisionObject note) {
        return new Translation2d(note.getX(), note.getY());
    }

}