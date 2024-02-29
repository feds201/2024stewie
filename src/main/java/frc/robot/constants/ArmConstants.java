package frc.robot.constants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;

public class ArmConstants {
   public static class ArmPIDForExternalEncoder {
      public static final double kP = 0.;
      public static final double kI = 0.;
      public static final double kD = 0;
      public static final double kF = 0;

      public static final double kRotationTolerance = 0.5;
      public static final double kIZone = Double.POSITIVE_INFINITY;
      public static final double kIMin = -0.05;
      public static final double kIMax = 0.05;

      public static final double kArmRotationFeederSetpoint = 5;
      public static final double kArmInnerWingSetpoint = 30;

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
   public static TalonFXConfiguration GetArmMotorConfiguration() {
      TalonFXConfiguration configs = new TalonFXConfiguration();

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

      SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = configs.SoftwareLimitSwitch;
      softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
      softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
      softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = 40 * kArmGearReduction;
      softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = 1 * kArmGearReduction;

      return configs;
   }
}
