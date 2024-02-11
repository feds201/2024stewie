package frc.robot.subsystems.vision_sys;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class vision_sys extends SubsystemBase {

  public vision_sys() {
    System.out.println("VisionSubsystem initiated");
  }

  public abstract void periodic();

  public abstract boolean CheckTarget();

  public abstract Translation2d GetTarget();



}
