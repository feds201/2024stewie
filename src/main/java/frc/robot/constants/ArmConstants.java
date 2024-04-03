package frc.robot.constants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;

public class ArmConstants {

   public static boolean DidJuiliaNotPressButton = true;

   public static class ArmPIDForExternalEncoder {
      public static final double kP = 0.195;
      public static final double kI = 0.015;
      public static final double kD = 0;
      public static final double kF = 0;

      public static final double kRotationTolerance = 1;
      public static final double kIZone = Double.POSITIVE_INFINITY;
      public static final double kIMin = -0.05;
      public static final double kIMax = 0.05;

      public static final double kArmRotationFeederSetpoint = 2;
      public static final double kArmInnerWingSetpoint = 30;
      public static  final double kAmpPosition = 29;

      public static PIDController GetArmPID() {
         PIDController pid = new PIDController(kP, kI, kD);
         pid.setTolerance(kRotationTolerance);
         pid.setIZone(kIZone);
         pid.setIntegratorRange(kIMin, kIMax);
         return pid;
      }
   }

   public static final double kArmSpeed = 0.05;
   public static final double kArmRotationDelay = 3;

   public static final double kArmGearReduction = 50;
   public static final double kHoldThreshold = 0.01;
   public static final double kArmSpeedScaler = 4;
   
   public static final double kArmClimbLimit = 78;


   public static TalonFXConfiguration GetArmMotorConfiguration() {
      TalonFXConfiguration configs = new TalonFXConfiguration();

      MotorOutputConfigs motorOutputConfigs = configs.MotorOutput;
      motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

      Slot0Configs slot0Configs = configs.Slot0;

      slot0Configs.kS = 0; // add 0.24 V to overcome friction //TODO: tune this
      slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
      // PID runs on position
      slot0Configs.kP = 0; // TODO: FOR SURE TUNE THESE
      slot0Configs.kI = 0;
      slot0Configs.kD = 0;

      MotionMagicConfigs motionMagicConfigs = configs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = 10; // TODO: FOR SURE TUNE THESE DAWG!!!
      motionMagicConfigs.MotionMagicAcceleration = 10;
      motionMagicConfigs.MotionMagicJerk = 10;

      return configs;
   }
}
