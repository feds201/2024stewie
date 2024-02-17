// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.OIConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.shooter.ShootNoteVoltage;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.distance_sensor.DistanceSensor;
import frc.robot.subsystems.distance_sensor.SensorManager;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.utils.Telemetry;

public class RobotContainer {
  

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConstants.MaxSpeed * 0.1).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  private Command runAuto = drivetrain.getAutoPath("4 ring path");

  private final Shooter shooter;
  private final Intake intake;
  private final Arm arm;
  private final Climber climber;
  private final SensorManager sensorManager;
 
   //private DistanceSensor currentSensor;

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  

  public RobotContainer() {
    
    sensorManager = new SensorManager();
    arm = new Arm();
    shooter = new Shooter(() -> arm.getArmAngle());
    climber = new Climber();
    intake = new Intake();

    driverController = new CommandXboxController(OIConstants.kDriverController);
    operatorController = new CommandXboxController(OIConstants.kOperatorController);
    configureBindings();    
    configureTestCommands();    
  }

  public void configureTestCommands() {
    // Display Subsystems on Shuffleboard under each of the tabs
    ShuffleboardTab commandsTab = Shuffleboard.getTab("commands");
    // commandsTab.add("Run Intake Wheels", new WaitCommand(10).alongWith(new ShootNoteVoltage(shooter, () -> shooter.getShootVoltage())));
  } 

  private void configureBindings() {
    // Swerve
     drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * SwerveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    driverController.a().onTrue(sensorManager.getMXP());
     driverController.b().onTrue(sensorManager.getOnboard());



    //driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //driverController.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    driverController.x().whileTrue(new ShootNoteVoltage(shooter, () -> 0.5));
  }
  
  public Command getAutonomousCommand() { 
    return runAuto;
  }
}