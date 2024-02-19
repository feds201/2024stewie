// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.RotateWristBasic;
import frc.robot.commands.arm.RotateArmBasic;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.shooter.RotateShooterBasic;
import frc.robot.commands.shooter.ShootNoteVelocity;
import frc.robot.commands.shooter.ShootNoteVoltage;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.utils.Telemetry;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConstants.MaxSpeed * 0.1).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1) // Add
                                                                                                                 // a
                                                                                                                 // 10%
                                                                                                                 // deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Shooter shooter;
  private final Intake intake;
  private final Arm arm;
  private final Climber climber;

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  ShuffleboardTab commandsTab = Shuffleboard.getTab("commands");

  public RobotContainer() {
    arm = new Arm();
    shooter = new Shooter(() -> arm.getArmAngle());
    climber = new Climber();
    intake = new Intake();

    commandsTab.add("arm", arm);
    // commandsTab.add("shooter", shooter);
    commandsTab.add("climber", climber);
    commandsTab.add("intake", intake);

    driverController = new CommandXboxController(OIConstants.kDriverController);
    operatorController = new CommandXboxController(OIConstants.kOperatorController);
    configureBindings();
    configureTestCommands();
  }

  public void configureTestCommands() {
    // Display Subsystems on Shuffleboard under each of the tabs
    commandsTab.add("Run Shooter Wheels",
        new WaitCommand(10)
            .alongWith(
                new ShootNoteVoltage(shooter, () -> ShooterConstants.kShootVoltage)));

    commandsTab.add("Rotate Shooter Simple",
        new WaitCommand(10)
            .alongWith(
                new RotateShooterBasic(shooter, ShooterConstants.kRotateSpeed) // 20% is WAY too fast
            ));

    commandsTab.add("Run Intake Wheels",
        new WaitCommand(10)
            .alongWith(
                new IntakeIn(intake, IntakeConstants.kWheelSpeed)));

    commandsTab.add("Rotate Intake Simple",
        new WaitCommand(10)
            .alongWith(
                new RotateWristBasic(intake, IntakeConstants.kRotateSpeed)));

    commandsTab.add("Rotate Intake Backwards Simple",
        new WaitCommand(10)
            .alongWith(
                new RotateWristBasic(intake, -IntakeConstants.kRotateSpeed)));

    commandsTab.add("Run Climber Simple",
        new WaitCommand(10)
            .alongWith(
                new ExtendClimber(climber, ClimberConstants.kClimberSpeed)));

    commandsTab.add("Rotate Arm Simple",
        new WaitCommand(10)
            .alongWith(
                new RotateArmBasic(arm, ArmConstants.kArmSpeed)));
  }

  private void configureBindings() {
    // Swerve

    final double headingTolerance = Math.toRadians(2.0); // Define a tolerance for heading alignment

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {
          // Calculate desired heading based on right stick direction
          Rotation2d desiredHeading = new Rotation2d(driverController.getRightX(), driverController.getRightY());

          // Calculate the difference between current heading and desired heading
          Rotation2d currentHeading = drivetrain.getRotation3d().toRotation2d();
          double headingDifference = desiredHeading.minus(currentHeading).getRadians();

          return drive.withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed) // Drive forward with negative Y
                                                                                       // (forward)
              .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed)
              .withRotationalRate(-driverController.getRightX() * SwerveConstants.MaxAngularRate); // Drive counterclockwise with negative X (left); // Drive left with negative X
                                                                                       // (left)
          // if (headingDifference > headingTolerance) {
          //   return drive.withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed) // Drive forward with negative
          //                                                                                // Y (forward)
          //       .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed) // Drive left with negative X
          //                                                                               // (left)
          //       .withRotationalRate(desiredHeading.minus(currentHeading).getRadians() * SwerveConstants.MaxAngularRate); // Drive
          //   // counterclockwise
          //   // with negative X
          //   // (left)
          // } else {
          //   return drive.withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed) // Drive forward with negative
          //                                                                                // Y (forward)
          //       .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed) // Drive left with negative X
          //                                                                               // (left)
          //       .withRotationalRate(0); // Drive counterclockwise with negative X (left)
          // }

        }));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain
        .applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    driverController.leftTrigger().whileTrue(new IntakeIn(intake, IntakeConstants.kWheelSpeed));

    operatorController.x().whileTrue(new ExtendClimber(climber, ClimberConstants.kClimberSpeed));
    operatorController.leftTrigger().whileTrue(new ShootNoteVelocity(shooter, () -> ShooterConstants.kShootVelocity));
    operatorController.leftTrigger().whileTrue(new ShootNoteVoltage(shooter, () -> ShooterConstants.kShootVoltage));
    operatorController.povUp().whileTrue(new RotateShooterBasic(shooter, ShooterConstants.kRotateSpeed));
  }

  public Command getAutonomousCommand() {
    return runAuto;
  }
}