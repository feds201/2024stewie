package frc.robot.subsystems.vision.Components;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class BackCamera extends Camera {
    public BackCamera() {}
    public void init(){
        System.out.println(Variables.BackCam.name +" initiating");
    }
    public void periodic(){
        System.out.println(Variables.BackCam.name +" periodic");
        Variables.BackCam.tv = table.getEntry("tv").getNumber(0).intValue();
        Variables.BackCam.tx = table.getEntry("tx").getNumber(0).doubleValue();
        Variables.BackCam.ty = table.getEntry("ty").getNumber(0).doubleValue();
        Variables.BackCam.ta = table.getEntry("ta").getNumber(0).doubleValue();
        Variables.BackCam.tid = table.getEntry("tid").getNumber(0).intValue();
        Variables.BackCam.CameraMode = table.getEntry("camMode").getNumber(0);
        SmartDashboard.putNumber("X-axis", Variables.BackCam.tx);
        SmartDashboard.putNumber("Y-axis", Variables.BackCam.ty);
        SmartDashboard.putNumber("Target ID", Variables.BackCam.tid);
        SmartDashboard.putBoolean("Target Locked", true);
        System.out.println(Variables.BackCam.name +" initiated");
    }
}
