// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.shooter.RotateShooter;
import frc.robot.commands.shooter.ShootNoteVelocity;
import frc.robot.commands.shooter.ShootNoteVoltage;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.Intake.IntakeIn;

import frc.robot.commands.Intake.RotateArm;
import frc.robot.commands.Intake.WristIn;
// import frc.robot.commands.Test.TestSensor;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.ArmRotationEncoder.ArmRotationEncoder;
// import frc.robot.subsystems.DistanceSensor.DistanceSensor;
// import frc.robot.subsystems.DistanceSensor.DistanceSensorMXP;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.utils.Telemetry;

public class RobotContainer {
  // private double MaxSpeed = 6; // 6 meters per second desired top speed
  // private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // /* Setting up bindings for necessary control of the swerve drive platform */
  // public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  //                                                              // driving in open loop
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // private final Telemetry logger = new Telemetry(MaxSpeed);

  // private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Shooter shooter;
  private final ArmRotationEncoder armRotationEncoder;
  
  private final Intake intake;
  //private final DistanceSensor distanceSensor;
  //private final DistanceSensorMXP distanceSensorMXP;

  
  private final CommandXboxController driverController;
  // private final CommandXboxController operatorController;

  public RobotContainer() {
    shooter = new Shooter();
    armRotationEncoder = new ArmRotationEncoder();
    intake = new Intake();

    //distanceSensor = new DistanceSensor();
    //distanceSensorMXP = new DistanceSensorMXP();
    driverController = new CommandXboxController(0);
    // operatorController = new CommandXboxController(1);
    configureBindings();    
  }

  private void configureBindings() {
    // // Swerve
    //  drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

    // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driverController.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // // reset the field-centric heading on left bumper press
    // driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    // drivetrain.registerTelemetry(logger::telemeterize);
    
    // Shooter
    driverController.leftBumper().whileTrue(new ShootNoteVoltage(shooter, () -> shooter.getShooterVoltage()));
    driverController.rightBumper().whileTrue(new ShootNoteVelocity(shooter, () -> shooter.getShooterVelocity())); 
    driverController.povDown().onTrue(new RotateShooter(shooter, shooter.getShooterAngle(), armRotationEncoder)); 
    driverController.b().whileTrue(new IntakeIn(intake, 0.1));
    
    driverController.a().whileTrue(new RotateArm(intake, 0.5));

    driverController.x().onTrue(new WristIn(intake, 5));

    driverController.y().onTrue(new WristIn(intake, 5).andThen(new IntakeIn(intake, 0.1)));
  }

  public Command getAutonomousCommand() { 
    return null;
  }
}
