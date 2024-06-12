package frc.robot.subsystems.Vision.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public record DashBoardManager() {

    public void DashBoard(String type, String name, Object value) {
        switch (type) {
            case "int":
                SmartDashboard.putNumber(name, (int) value);
                break;
            case "double":
                SmartDashboard.putNumber(name, (double) value);
                break;
            case "String":
                SmartDashboard.putString(name, (String) value);
                break;
            case "boolean":
                SmartDashboard.putBoolean(name, (boolean) value);
                break;
            default:
                SmartDashboard.putString("Error", "Invalid type");
                break;
        }


    }

    public void DashBoard(String title, double x, double y, boolean targetLocked, double[] angle) {
        SmartDashboard.putNumber(title + " " + "Xaxis", x);
        SmartDashboard.putNumber(title + " " + "Yaxis", y);
        SmartDashboard.putBoolean(title + " " + "Target Locked", targetLocked);
        SmartDashboard.putNumberArray(title + " " + "Angle", angle);
    }
}
