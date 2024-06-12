
package frc.robot.constants;

public class SwerveConstants {
    public static final double MaxSpeed = 5; // 6 meters per second desired top speed
    public static final double MaxAngularRate = 1 * Math.PI; // 3/4 of a rotation per second max angular velocity
    public static final double kAutonMoveSpeed = 1;
    public static final double kAlignmentOutput = 0.01;
		public static double kAlignmentTolerance = 1;
		public static double kStuckTolerance = -10;
		
		//this is for the horizontal yaw align to april tag control
		public static double kRotationP = 0.085;
		//0.07
		public static double kRotationI = .000;
		//.0001
		public static double kRotationD = .00;
		public static double speedpercentage = 1.0;
}
