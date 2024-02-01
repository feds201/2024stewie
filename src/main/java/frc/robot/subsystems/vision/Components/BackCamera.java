package frc.robot.subsystems.vision.Components;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class BackCamera extends Camera {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-notes");
    public BackCamera() {}
    public void init(){
        System.out.println(Variables.BackCam.name +" initiating");

    }
    public void BackCamPeroidic(){
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
}
