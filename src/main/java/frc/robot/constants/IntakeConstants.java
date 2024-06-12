package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;

public class IntakeConstants {
  public static class WristPID {
    public final static double kP = 0.003;//0.002
    public final static double kI = 0.001;
    public final static double kD = 0.0;
    public final static double kIZone = Double.POSITIVE_INFINITY;
    public final static double kRotationTolerance = 10;
    public static final double kSpitOutPosition = 201;
    public final static double kWristNotePosition = 308;
    public final static double kWristIdlePosition = 88;
    public  final static  double kAmpPosition = 223;
    public final static double kWristShooterFeederSetpoint = 94; // TODO: Ideally all of the above positions should be based on this "home" position so we only have to change this
    public static double kWristOutOfTheWay = 150;
      
      public static PIDController GetWristPID() {
      PIDController pid = new PIDController(kP, kI, kD);
      pid.setIZone(kIZone);
      pid.setTolerance(kRotationTolerance);
      return pid;
    }
    public static PIDController GetWristAmpPID() {
      PIDController pid = new PIDController(0.003, 0.00, 0.0005);
      pid.setIZone(kIZone);
      pid.setTolerance(kRotationTolerance);
      return pid;
    }
  }

  public static final double kIntakeNoteWheelSpeed = -1;//.6
  public static final double kSpitOutNoteWheelSpeed = 1;
  public static final double kAmpInWheelSpeed = 1;

  public static final double kHandoffNoteWheelSpeed = 0.6;
  public static final double kDistanceSensorDetectedDelay = 0.1;
  
}
