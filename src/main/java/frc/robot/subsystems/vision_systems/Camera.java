package frc.robot.subsystems.vision_systems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Camera extends SubsystemBase {
    public abstract NetworkTable getTable();
    public abstract void periodic();
    
}