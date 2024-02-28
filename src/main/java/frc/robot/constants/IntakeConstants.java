package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;

public class IntakeConstants {
  public static class WristPID {
    public final static double kP = 0.0;
    public final static double kI = 0.0;
    public final static double kD = 0.0;
    public final static double kIZone = Double.POSITIVE_INFINITY;
    public final static double kRotationTolerance = 0.5;

    public static PIDController GetWristPID() {
      PIDController pid = new PIDController(kP, kI, kD);
      pid.setIZone(kIZone);
      pid.setTolerance(kRotationTolerance);
      return pid;
    }
  }
  
  public final static double kWristNotePosition = 10;
  public final static double kWristShooterFeederSetpoint = 10;

  public static final double kWheelSpeed = -0.25;
  public static final double kRotateSpeed = 0.20;
}
