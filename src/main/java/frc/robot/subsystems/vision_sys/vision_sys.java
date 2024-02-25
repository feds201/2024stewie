package frc.robot.subsystems.vision_sys;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision_sys.utils.VisionObject;

public abstract class vision_sys extends SubsystemBase {



  public vision_sys() {
    SmartDashboard.putBoolean("isBlueAlliance", false);
  }

  public abstract void periodic();

  public abstract boolean CheckTarget();

  public abstract Translation2d GetTarget(VisionObject object);

}
