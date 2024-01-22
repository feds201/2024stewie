package frc.robot.subsystems.vision.Components;

import edu.wpi.first.networktables.NetworkTable;

public class BackCamera extends Camera {
    public BackCamera() {
        super();
        moduleName = "limelight";
    }

    public void Periodic() {
        NetworkTable table = this.table.getTable(moduleName);
        pipelineName = table.getEntry("pipeline").getDouble(0);
        ledMode = table.getEntry("ledMode").getDouble(0);
        tx = table.getEntry("tx").getDouble(0);
        ty = table.getEntry("ty").getDouble(0);
        ta = table.getEntry("ta").getDouble(0);
        tv = table.getEntry("tv").getBoolean(false);
    }

    public String getModuleName() {
        return "moduleName";
    }
}