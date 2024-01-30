package frc.robot.subsystems.vision.Components;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class FrontCamera extends Camera {
    public FrontCamera() {
        super();
    }
    public void init(){
        System.out.println(Variables.FrontCam.name +" initiating");
    }
    public void periodic(){
        System.out.println(Variables.FrontCam.name +" periodic");
        Variables.FrontCam.tv = table.getEntry("tv").getNumber(0).intValue();
        Variables.FrontCam.tx = table.getEntry("tx").getNumber(0).doubleValue();
        Variables.FrontCam.ty = table.getEntry("ty").getNumber(0).doubleValue();
        Variables.FrontCam.ta = table.getEntry("ta").getNumber(0).doubleValue();
        Variables.FrontCam.tid = table.getEntry("tid").getNumber(0).intValue();
        Variables.FrontCam.CameraMode = table.getEntry("camMode").getNumber(0);
        SmartDashboard.putNumber("X-axis", Variables.FrontCam.tx);
        SmartDashboard.putNumber("Y-axis", Variables.FrontCam.ty);
        SmartDashboard.putNumber("Target ID", Variables.FrontCam.tid);
        SmartDashboard.putBoolean("Target Locked", true);
        System.out.println(Variables.FrontCam.name +" initiated");
    }
}
