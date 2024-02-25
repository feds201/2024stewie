package frc.robot.subsystems.vision_sys.LockTag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.vision_sys.VisionVariables;

public class AprilTagLock implements RotationSource {

    NetworkTable table = NetworkTableInstance.getDefault().getTable(CameraConstants.BackCam.BACK_CAMERA_NETWORK_TABLES_NAME);
    public static PIDController rotationPID = createPIDController();
    private static PIDController createPIDController() {
        PIDController pid = new PIDController(.01, .02, .001);
        pid.setTolerance(.25); // allowable angle error
        pid.enableContinuousInput(0, 360); // it is faster to go 1 degree from 359 to 0 instead of 359 degrees
        pid.setSetpoint(0); // 0 = apriltag angle
        return pid;
    }
    @Override
    public double getR() {
        return rotationPID.calculate(table.getEntry("tx").getDouble(0));
    }
}