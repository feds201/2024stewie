package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;

public class ShooterConstants {
    public static class RotationPIDForExternalEncoder {
        public static final double kRotateP = 0.015;
        public static final double kRotateI = 0.001;
        public static final double kRotateD = 0;

        public static final double kRotateTolerance = 0.5;
        public static final double kRotateIZone = Double.POSITIVE_INFINITY;
        public static final double kIMax = 0.05;
        public static final double kIMin = -0.05;

        public static final double kShooterHorizontal = -55;
        public static final double kArmSubwooferSetpoint = -10; // 7 feet 10 inches
        public static final double kArm60InchSetpoint = -15; // 5 feet away
        public static final double kShooterRotationFeederSetpoint = -30;

        public static PIDController GetRotationPID() {
            PIDController pid = new PIDController(kRotateP, kRotateI,
                    kRotateD);

            pid.setTolerance(kRotateTolerance);
            pid.setIZone(kRotateIZone);
            pid.setIntegratorRange(kIMin, kIMax);
            return pid;
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

    public static final double A = -2.0714;
    
    public static final double B = 17.899;
    
    public static final double C = -3.8765;
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


