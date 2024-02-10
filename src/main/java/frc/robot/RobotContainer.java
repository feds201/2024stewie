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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.vision.TurnTowardsNote;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
<<<<<<< HEAD
=======
import frc.robot.subsystems.vision.Constants.Variables;
>>>>>>> parent of cfafc63 (Update vision subsystem and RobotContainer)
import frc.robot.utils.Telemetry;

public class RobotContainer {
  // private final DistanceSensor s_DistanceSensor;
  private final VisionSubsystem vision_sys;
  private final double MaxSpeed = 6; // 6 meters per second desired top speed
  private final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  public RobotContainer() {
    // s_DistanceSensor = new DistanceSensor();
    vision_sys = new VisionSubsystem();
    configureBindings();
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
            drivetrain.
                    applyRequest(
                            () -> drive
                                    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
                    )
    );

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
<<<<<<< HEAD
    joystick.b().whileTrue(
            drivetrain.applyRequest(
                    () -> point
                            .withModuleDirection(
                                    new Rotation2d(
                                            -joystick.getLeftY(),
                                            -joystick.getLeftX()
                                    )
                            )
            )
    );

//    joystick.x().whileTrue(drivetrain.applyRequest(() -> (drive.withVelocityX(ExportedVariables.Velocity[0]).withVelocityY(ExportedVariables.Velocity[1]).withRotationalRate(ExportedVariables.Velocity[2]) point.withModuleDirection(new Rotation2d(ExportedVariables.Velocity[2], ExportedVariables.Velocity[2]))));
    joystick.x().whileTrue(new TurnTowardsNote(drivetrain, vision_sys));

=======
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    
    joystick.x().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(Variables.ExportedVariables.Velocity[0], Variables.ExportedVariables.Velocity[1]))));
>>>>>>> parent of cfafc63 (Update vision subsystem and RobotContainer)
            


    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
