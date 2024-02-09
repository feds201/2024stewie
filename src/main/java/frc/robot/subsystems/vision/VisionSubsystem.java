// package frc.robot.subsystems.vision;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CameraVariables;
// import frc.robot.subsystems.vision.Components.Camera;

// public class VisionSubsystem extends SubsystemBase {
//     private static final double MaxSpeed = 1.5;
    
//     private static final double MaxAngularRate = 1;
//     private final Camera camera;

//     public VisionSubsystem() {
//         camera = new Camera();
//         System.out.println("VisionSubsystem initiated");
//     }

    
//     private double[] getVelocity(Translation2d noteCords) {
//         double deadband = 7.0; // Adjust the deadband value as needed
//         double x = noteCords.getX();
//         double[] velocity = new double[3];

//         if (Math.abs(x) < deadband) {
//             velocity[0] = 0;
//             velocity[1] = 0;
//             velocity[2] = 0;
//         } else if (x > 0) {
//             velocity[0] = 0.5 * MaxSpeed;
//             velocity[1] = 0.5 * MaxSpeed;
//             velocity[2] = 0.1 * MaxAngularRate;
//         } else {
//             velocity[0] = -0.5 * MaxSpeed;
//             velocity[1] = -0.5 * MaxSpeed;
//             velocity[2] = -0.1 * MaxAngularRate;
//         }
//             SmartDashboard.putNumber("Velocity X", velocity[0]);
//             SmartDashboard.putNumber("Velocity Y", velocity[1]);
//             SmartDashboard.putNumber("Rotational", velocity[2]);

//         return velocity;
//     }

 

//     public void periodic() {
//         camera.periodic();
//         boolean targetLocked = camera.checkNote();
//         SmartDashboard.putBoolean("Target Locked", targetLocked);
//         if (targetLocked) {
//             Translation2d targetLocation = camera.getTargetLocation();
//             SmartDashboard.putNumber("Target X", targetLocation.getX());
//             SmartDashboard.putNumber("Target Y", targetLocation.getY());
//             // var hi = getVelocity(targetLocation);
//             CameraVariables.CalculatedVariables.Velocity = ( getVelocity(targetLocation)) ;

            
//         }

//     }

//     public Translation2d getTargetLocation() {
//         return camera.getTargetLocation();
//     }

//     public boolean checkNote() {
//         return camera.checkNote();
//     }
// }