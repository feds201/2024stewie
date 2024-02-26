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
import frc.robot.subsystems.vision_sys.VisionVariables.ExportedVariables;

import java.util.Random;

public class BackCamera extends vision_sys {
    public static DashBoardManager dashBoardManager;
    public static String nt_key;
    public static NetworkTable table;
    public static VisionObject tag;
    public Random random = new Random();

    public BackCamera() {
        dashBoardManager = new DashBoardManager();
        nt_key = CameraConstants.BackCam.BACK_CAMERA_NETWORK_TABLES_NAME;
        table = NetworkTableInstance.getDefault().getTable(nt_key);
        tag = new VisionObject(0, 0, 0, ObjectType.APRILTAG);
    }
    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        tag.update(
                random.nextDouble() * 100,
                random.nextDouble() * 100,
                random.nextDouble() * 360
        );


    }
    @Override
    public void periodic() {
        tag.update(
                                table.getEntry("tx").getNumber(0).doubleValue(),
                                table.getEntry("ty").getNumber(0).doubleValue(),
                                table.getEntry("ta").getNumber(0).doubleValue()
        );
        VisionVariables.BackCam.tv = table.getEntry("tv").getNumber(0).intValue();

        dashBoardManager.DashBoard(
                "BackCamera",
                tag.getX(),
                tag.getY(),
                CheckTarget(),
                tag.getAngle()
        );
        SmartDashboard.putNumber("Estimated Shooter Angle",  setShooterAngle(tag));

        if (CheckTarget()) {
            VisionVariables.BackCam.target = tag;
            VisionVariables.ExportedVariables.AngleForShooter = setShooterAngle(tag);
            VisionVariables.ExportedVariables.Distance = tag.getDistance();
            SmartDashboard.putNumber("Distance", VisionVariables.ExportedVariables.Distance);
        }

        SmartDashboard.putNumber("limelight distance", ExportedVariables.Distance);
        SmartDashboard.putNumber("tz", table.getEntry("TZ").getNumber(0).doubleValue());

    }
    @Override
    public boolean CheckTarget() {
        int target = table.getEntry("tid").getNumber(0).intValue();
        return target == 4;
    }
    @Override
    public Translation2d GetTarget(VisionObject note) {
        return null;
    }
    private int setShooterAngle(VisionObject tag) {
        return (int) tag.getAngle()[1];
    }




}