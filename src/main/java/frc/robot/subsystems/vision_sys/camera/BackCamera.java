package frc.robot.subsystems.vision_sys.camera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.vision_sys.VisionVariables;
import frc.robot.subsystems.vision_sys.utils.DashBoardManager;
import frc.robot.subsystems.vision_sys.utils.ObjectType;
import frc.robot.subsystems.vision_sys.utils.VisionObject;
import frc.robot.subsystems.vision_sys.vision_sys;

import java.util.Random;

public class BackCamera extends vision_sys {
    public static DashBoardManager dashBoardManager;
    public static String nt_key;
    public static NetworkTable table;
    public static VisionObject tag;
    public Random random = new Random();
//    public static PIDController rotationPID = createPIDController();

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
        Periodic();
    }

    @Override
    public void periodic() {
        tag.update(
                table.getEntry("tx").getNumber(0).doubleValue(),
                table.getEntry("ty").getNumber(0).doubleValue(),
                table.getEntry("ta").getNumber(0).doubleValue()
        );
        VisionVariables.BackCam.tv = table.getEntry("tv").getNumber(0).intValue();
        Periodic();

    }

    @Override
    public boolean CheckTarget() {
        int target = table.getEntry("tid").getNumber(0).intValue();
        if (target == 4){return true;}else return target== 7;
    }

    public Translation2d GetTarget(VisionObject note) {
        return null;
    }
    private int setShooterAngle(VisionObject tag) {
        return (int) tag.getAngle()[1];
    }

    private void Periodic(){
        dashBoardManager.DashBoard(
                "BackCamera",
                tag.getX(),
                tag.getY(),
                CheckTarget(),
                tag.getAngle()
        );
        SmartDashboard.putNumber("Estimated Shooter Angle",  setShooterAngle(tag));
//        try {
//            VisionVariables.ExportedVariables.pid_value_for_Alignmen = rotationPID.calculate(VisionVariables.BackCam.target.getX());
//        } catch (Exception e) {
//            e.printStackTrace();
//        }

        if (CheckTarget()) {
            VisionVariables.BackCam.target = tag;
            VisionVariables.ExportedVariables.AngleForShooter = setShooterAngle(tag);
            VisionVariables.ExportedVariables.Distance = tag.getDistance();
            SmartDashboard.putNumber("Distance", VisionVariables.ExportedVariables.Distance);
        }
    }
//    private static PIDController createPIDController() {
//        PIDController pid = new PIDController(.05, .025, .001);
//        Shuffleboard.getTab("swerve").add("april tag pid", pid);
//        pid.setTolerance(5); // allowable angle error
//        pid.enableContinuousInput(0, 180); // it is faster to go 1 degree from 359 to 0 instead of 359 degrees
//        pid.setSetpoint(0); // 0 = apriltag angle
//        return pid;
//    }


}