// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.shooter.RotateShooter;
import frc.robot.commands.shooter.ShootNoteVelocity;
import frc.robot.commands.shooter.ShootNoteVoltage;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.WristIn;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.utils.Telemetry;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Shooter shooter;
  private final Intake intake;
  private final Arm arm;
  private final Climber climber;

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  public RobotContainer() {
    arm = new Arm();
    shooter = new Shooter(() -> arm.getArmAngle());
    intake = new Intake();
    climber = new Climber();

    driverController = new CommandXboxController(OIConstants.kDriverController);
    operatorController = new CommandXboxController(OIConstants.kOperatorController);
    configureBindings();    
  }

  private void configureBindings() {
    // Swerve
     drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
    
    // Shooter
    operatorController.rightBumper().whileTrue(new ShootNoteVelocity(shooter,  () -> shooter.getShooterVelocity()));
    operatorController.povLeft().whileTrue(new ShootNoteVoltage(shooter, () -> shooter.getShooterVoltage()));
    operatorController.povDown().onTrue(new RotateShooter(shooter, shooter.getShooterAngle()));

    // Intake
    operatorController.povUp().whileTrue(new IntakeIn(intake, 0.1));
    //driverController.b().whileTrue( new RotateArm(intake, 0.5, armRotationEncoder));
    operatorController.y().onTrue(new WristIn(intake, 5.0));
    operatorController.povRight().onTrue(new WristIn(intake, 5).andThen(new IntakeIn(intake, 0.1)));
  }
  
  public Command getAutonomousCommand() { 
    return runAuto;
  }
}