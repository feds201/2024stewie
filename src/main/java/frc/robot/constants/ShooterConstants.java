package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ShooterConstants {
    public static class Rotation {

        // PID CONSTANTS
        // public static final double kP = 0.012;
        // public static final double kI = 0.001;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kGoalTolerance = 0.5;
        public static final double kIZone = Double.POSITIVE_INFINITY;
        public static final double kIMax = 0.05;
        public static final double kIMin = -0.05;

        public static final double kS = 0.022;
        public static final double kG = 0.02;
        public static final double kV = 0;
        public static final double kA = 0;

        // // TRAPEZOIDAL PROFILE CONSTANTS
        // public static final double kMaxSpeed = 0.1;
        // public static final double kMaxAcceleration = 4;

        // SETPOINTS

        public static final double kArmSubwooferSetpoint = -10; // 7 feet 10 inches
        public static final double kArm60InchSetpoint = -15; // 5 feet away
        public static final double kShooterRotationFeederSetpoint = -26;

        public static PIDController GetRotationPID() {
            PIDController pid = new PIDController(kP, kI, kD);

            pid.setTolerance(kGoalTolerance);
            pid.setIZone(kIZone);
            pid.setIntegratorRange(kIMin, kIMax);

            return pid;
        }

        public static ArmFeedforward GetRotationFF() {
            ArmFeedforward rotationFF = new ArmFeedforward(kS, kG, kV, kA);
            return rotationFF;
        }
    }

    public static final int kServoThickSideSpeed = 0;
    public static final int kServoThinSideSpeed = 1;

    public static final double kShootVelocity = -100;
    public static final double kShootVoltage = 0.1;

    public static final double kRotateSpeed = 0.03;
    public static final double kRotateShooterDelay = 0;

    public static final int kThickWheelServoPort = 1;
    public static final int kThinWheelServoPort = 2;
    public static final double kHandoffDelay = 0;

    public static TalonFXConfiguration GetWheelsConfiguration() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        /*
         * Voltage-based velocity requires a feed forward to account for the back-emf of
         * the motor
         */
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                 // volts / Rotation per second

        configs.MotionMagic.MotionMagicCruiseVelocity = 80;
        configs.MotionMagic.MotionMagicAcceleration = 160;
        configs.MotionMagic.MotionMagicJerk = 1600;

        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 0.9 * 12;
        configs.Voltage.PeakReverseVoltage = -0.9 * 12;

        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        return configs;
    }

    public static TalonFXConfiguration GetRotationConfiguration() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        return configs;
    }
}

/*
 * Full Motor Configuration Old Code
 * 
 * TalonFXConfiguration configs = new TalonFXConfiguration();
 * 
 * 
 * Voltage-based velocity requires a feed forward to account for the back-emf of
 * the motor
 * 
 * configs.Slot0.kP = 0.0; // An error of 1 rotation per second results in 2V
 * output
 * configs.Slot0.kI = 0.0; //An error of 1 rotation per second increases output
 * by 0.5V every second
 * configs.Slot0.kD = 0.000; // A change of 1 rotation per second squared
 * results in 0.01 volts output
 * configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333
 * rps per V, 1/8.33 = 0.12
 * // volts / Rotation per second
 * // configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in
 * 2V output
 * // configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases
 * output by 0.5V every second
 * // configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared
 * results in 0.01 volts output
 * // configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V =
 * 8.333 rps per V, 1/8.33 = 0.12
 * // // volts / Rotation per second
 * // Peak output of 8 volts
 * configs.Voltage.PeakForwardVoltage = 8;
 * configs.Voltage.PeakReverseVoltage = -8;
 * 
 * //
 * // Torque-based velocity does not require a feed forward, as torque will
 * // accelerate the rotor up to the desired velocity by itself
 * //
 * // configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5
 * amps output
 * // configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases
 * output by 0.1 amps every second
 * // configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared
 * results in 1 amp output
 * 
 * // // Peak output of 40 amps
 * configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
 * configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
 * 
 * shooterTopMain.getConfigurator().apply(configs);
 */
