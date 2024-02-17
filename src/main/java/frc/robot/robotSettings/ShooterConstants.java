package frc.robot.robotSettings;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ShooterConstants {

    public static final double kRotateP = 0;
    public static final double kRotateI = 0;
    public static final double kRotateD = 0;

    public static final double kRotateTolerance = 0.5;
    public static final double kRotateIZone = Double.POSITIVE_INFINITY;


    public static TalonFXConfiguration GetShooterConfiguration() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Slot0.kP = 0.0; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.000; // A change of 1 rotation per second squared results in 0.01 volts output
        /*
        * Voltage-based velocity requires a feed forward to account for the back-emf of
        * the motor
        */
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                // volts / Rotation per second

        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;

        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        return configs;
    }
}

/* Full Motor Configuration Old Code
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