package frc.robot.constants;

public class ArmConstants {
   public static final double kP = 0.012;
   public static final double kI = 0.002;
   public static final double kD = 0;
   public static final double kF = 0;

   public static final double kArmSpeed = 0.05;

   public static final double kArmRotationFeederSetpoint = 5;
   public static final double kArmInnerWingSetpoint = 30;

   public static final double kRotationTolerance = 0.5;
   // TODO: Do we need a velocity tolerance?
   public static final double kIZone = Double.POSITIVE_INFINITY;
   public static final double kIMin  = -0.05;
   public static final double kIMax  =  0.05;
   public static final double kArmRotationDelay = 3;

}
