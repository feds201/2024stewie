
// package frc.robot.commands.vision;


// import org.ejml.equation.Variable;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.CameraVariables;
// import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
// import frc.robot.subsystems.vision.VisionSubsystem;
// import frc.robot.subsystems.vision_systems.BackCamera;

// public class RotateUntilDetectionCommand extends Command {
//     private final CommandSwerveDrivetrain drivetrain;
//     private final VisionSubsystem vision_sys;
//     private final double rotationIncrement;


    
//     float Kp = -0.1f;  // Proportional control constant
//     public RotateUntilDetectionCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision_sys, double rotationIncrement) {
//         this.drivetrain = drivetrain;
//         this.vision_sys = vision_sys;

//         this.rotationIncrement = rotationIncrement;
//         addRequirements(drivetrain);
//     }

//     @Override
//     public void execute() {
//         double steering_adjust = Kp * CameraVariables.BackCam.tv;
        
//         new SwerveRequest.FieldCentric().withRotationalRate(steering_adjust);
//     }

//     @Override
//     public boolean isFinished() {
//         return !vision_sys.checkNote();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
//     }
// }