//package frc.robot.utils;
//
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//
//public class KalmanFilter1D {
//    private double Q; // Process noise covariance
//    private double R; // Measurement noise covariance
//    private double x; // Estimated value
//    private double P; // Estimation error covariance
//
//    public KalmanFilter1D(double Q, double R, double initialValue, double initialErrorCovariance) {
//        this.Q = Q;
//        this.R = R;
//        this.x = initialValue;
//        this.P = initialErrorCovariance;
//    }
//
//    public double filter(double measurement) {
//        // Prediction update
//        double x_pred = x;
//        double P_pred = P + Q;
//
//        // Measurement update
//        double K = P_pred / (P_pred + R);
//        x = x_pred + K * (measurement - x_pred);
//        P = (1 - K) * P_pred;
//
//        SmartDashboard.putNumber("TRIG: Original Distance Reading", measurement);
//        SmartDashboard.putNumber("TRIG: Filtered Distance Reading", x);
//        return x;
//    }
//
//}