package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;

public class IntakeConstants {
  public static class WristPID {
    public final static double kP = 0.0055;
    public final static double kI = 0.0015;
    public final static double kD = 0.0;
    public final static double kIZone = Double.POSITIVE_INFINITY;
    public final static double kRotationTolerance = 5;

    public static final double kSpitOutPosition = 205;
    public final static double kWristNotePosition = 295;
    public final static double kWristIdlePosition = 170;
    public final static double kWristShooterFeederSetpoint = 86; // Problem?

    public static PIDController GetWristPID() {
      PIDController pid = new PIDController(kP, kI, kD);
      pid.setIZone(kIZone);
      pid.setTolerance(kRotationTolerance);
      return pid;
    }
  }

  public static final double kWheelSpeed = -0.25;
  public static final double kRotateSpeed = 0.20;
  // public static final double kDistanceSensorDetectedDelay = 0.1;
}
