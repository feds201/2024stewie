package frc.robot.subsystems.vision.Components;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.Constants.Variables;

public class BackCamera extends Camera {
    private static final String nt_key = Variables.backCam_EName;
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable(nt_key);
    public BackCamera() {}

    public void periodic() {
        System.out.println(Variables.BackCam.name +" periodic");
        Variables.BackCam.tx = table.getEntry("tx").getNumber(0).doubleValue();
        Variables.BackCam.ty = table.getEntry("ty").getNumber(0).doubleValue();
        Variables.BackCam.ta = table.getEntry("ta").getNumber(0).doubleValue();
        Variables.BackCam.CameraMode = table.getEntry("camMode").getNumber(0);
        SmartDashboard.putNumber("Xaxis", Variables.BackCam.tx);
        SmartDashboard.putNumber("Yaxis", Variables.BackCam.ty);
        SmartDashboard.putNumber("Target ID", Variables.BackCam.tid);
        SmartDashboard.putBoolean("Target Locked", true);
        System.out.println(Variables.BackCam.name +" initiated");
    }

    public boolean checkNote() {
        return Variables.BackCam.ty != 0;
    }
}
