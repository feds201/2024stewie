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
import frc.robot.constants.OIConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.commands.arm.RotateArm;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.RotateWristBasic;
import frc.robot.commands.intake.RotateWristPID;
import frc.robot.commands.shooter.RotateShooter;
import frc.robot.commands.shooter.RotateShooterBasic;
import frc.robot.commands.shooter.ShootNoteVelocity;
import frc.robot.commands.shooter.ShootNoteVoltage;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.shooter.ShooterWheels;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.vision_sys.camera.BackCamera;
import frc.robot.subsystems.vision_sys.camera.FrontCamera;
import frc.robot.utils.Telemetry;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1) // Add
                                                                          // a
                                                                          // 10%
                                                                          // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

    private Command runAuto = drivetrain.getAutoPath("Tests");

    private final ShooterWheels shooterWheels;
    private final ShooterRotation shooterRotation;
    private final Wrist wrist;
    private final IntakeWheels intakeWheels;
    private final Arm arm;
    private final Climber climber;
    private final FrontCamera frontCamera;
    private final BackCamera backCamera;
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    ShuffleboardTab commandsTab = Shuffleboard.getTab("commands");

    public RobotContainer() {
        arm = new Arm();
        shooterWheels = new ShooterWheels();
        shooterRotation = new ShooterRotation(arm::getArmAngle);
        climber = new Climber();
        wrist = new Wrist();
        intakeWheels = new IntakeWheels();
        frontCamera = new FrontCamera();
        backCamera = new BackCamera();

        arm.getShuffleboardTab().add("arm", arm);
        shooterWheels.getShuffleboardTab().add("shooter wheels", shooterWheels);
        shooterRotation.getShuffleboardTab().add("shooter rotation", shooterRotation);
        climber.getShuffleboardTab().add("climber", climber);
        wrist.getShuffleboardTab().add("wrist", wrist);
        intakeWheels.getShuffleboardTab().add("wheels", intakeWheels);

        driverController = new CommandXboxController(OIConstants.kDriverController);
        operatorController = new CommandXboxController(OIConstants.kOperatorController);
        configureBindings();
        configureTestCommands();

        // shooter.setDefaultCommand(new RotateShooter(arm, () ->
        // -ShooterConstants.kRotateSpeed));
    }

    public void configureTestCommands() {
        shooterWheels.getShuffleboardTab().add("Run Shooter velocity",
                new WaitCommand(10)
                        .alongWith(
                                new ShootNoteVelocity(shooterWheels,
                                        () -> ShooterConstants.kShootVelocity)));

        shooterRotation.getShuffleboardTab().add(
                "Rotate Shooter PID",
                new WaitCommand(10)
                        .alongWith(
                                new RotateShooter(
                                        shooterRotation,
                                        ShooterConstants.kArmInnerWingSetpoint
                                )
                        )
        );

        intakeWheels.getShuffleboardTab().add(
                "Run Intake Wheels",
                new WaitCommand(10)
                        .alongWith(
                                new IntakeIn(
                                        intakeWheels,
                                        IntakeConstants.kWheelSpeed
                                )
                        )
        );

        wrist.getShuffleboardTab().add(
                "Rotate Intake Simple",
                new WaitCommand(10)
                        .alongWith(
                                new RotateWristBasic(
                                        wrist,
                                        IntakeConstants.kRotateSpeed
                                )
                        )
        );

        wrist.getShuffleboardTab().add(
                "Rotate Intake Backwards Simple",
                new WaitCommand(10)
                        .alongWith(
                                new RotateWristBasic(
                                        wrist,
                                        -IntakeConstants.kRotateSpeed
                                )
                        )
        );

        wrist.getShuffleboardTab().add(
                "Rotate Intake PID",
           new WaitCommand(10)
                        .alongWith(
                                new RotateWristPID(
                                        wrist,
                                        IntakeConstants.kWristNotePosition
                                )
                        )
        );

        climber.getShuffleboardTab().add(
                "Run Climber Simple",
                new WaitCommand(10)
                        .alongWith(
                                new ExtendClimber(
                                        climber,
                                        ClimberConstants.kClimberSpeed
                                )
                        )
        );


        arm.getShuffleboardTab().add("Rotate Arm",
                new WaitCommand(10)
                        .alongWith(
                                new RotateArm(
                                        arm,
                                        ArmConstants.kArmInnerWingSetpoint
                                )
                        )
        );

    }

    private void configureBindings() {
        // Swerve

        final double headingTolerance = Math.toRadians(2.0); // Define a tolerance for heading alignment

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        {
                            Rotation2d desiredHeading = new Rotation2d(
                                    driverController.getRightX(),
                                    driverController.getRightY()
                            );
                            Rotation2d currentHeading = drivetrain.getRotation3d().toRotation2d();
                            double headingDifference = desiredHeading.minus(currentHeading).getRadians();

                            return drive
                                    .withVelocityX(-driverController.getLeftY() * SwerveConstants.MaxSpeed)
                                    .withVelocityY(-driverController.getLeftX() * SwerveConstants.MaxSpeed)
                                    .withRotationalRate(-driverController.getRightX() * SwerveConstants.MaxAngularRate);
                        }
                )
        );

        driverController.a()
                .whileTrue(
                        drivetrain.applyRequest(() -> brake
                        )
                );

        driverController.b()
                .whileTrue(drivetrain
                    .applyRequest(() -> point
                            .withModuleDirection(
                                    new Rotation2d(
                                            -driverController.getLeftY(),
                                            -driverController.getLeftX()
                                    )
                            )
                    )
                );

        // reset the field-centric heading on left bumper press
        driverController.leftBumper()
                .onTrue(
                        drivetrain.runOnce(
                                drivetrain::seedFieldRelative
                        )
                );

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain
                .registerTelemetry(
                        logger::telemeterize
                );
        driverController.leftTrigger()
                .whileTrue(
                        new IntakeIn(
                                intakeWheels,
                                IntakeConstants.kWheelSpeed
                        )
                );
        operatorController.x()
                .whileTrue(
                        new ExtendClimber(
                                climber,
                                ClimberConstants.kClimberSpeed
                        )
                );
        operatorController.leftTrigger()
                .whileTrue(
                        new ShootNoteVelocity(
                                shooterWheels,
                                () -> ShooterConstants.kShootVelocity
                        )
                );
        operatorController.leftTrigger()
                .whileTrue(
                        new ShootNoteVoltage(
                                shooterWheels,
                                () -> ShooterConstants.kShootVoltage
                        )
                );
        operatorController.povUp()
                .whileTrue(
                        new RotateShooterBasic(
                                shooterRotation,
                                ShooterConstants.kRotateSpeed
                        )
                );
    }

    public Command getAutonomousCommand() {
        return runAuto;
    }
}