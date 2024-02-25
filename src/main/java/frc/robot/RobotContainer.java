// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.OIConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.commands.arm.RotateArm;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.RotateWristBasic;
import frc.robot.commands.intake.RotateWristPID;
import frc.robot.commands.shooter.RotateFeeder;
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
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.shooter.ShooterWheels;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.sensors.BreakBeamSensor;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
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
    private final ShooterFeeder servoThinSide; // These designations refer to the wheels on the intake being more on one
                                               // side (thick) and less on the other (thin)
    private final ShooterFeeder servoThickSide;
    private final Wrist wrist;
    private final IntakeWheels intakeWheels;
    private final Arm arm;
    private final Climber climber;
    private final BreakBeamSensor breakBeamSensor;

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    ShuffleboardTab commandsTab = Shuffleboard.getTab("commands");

    public RobotContainer() {
        arm = new Arm();
        shooterWheels = new ShooterWheels();
        shooterRotation = new ShooterRotation(() -> arm.getArmAngle());
        climber = new Climber();
        wrist = new Wrist();
        intakeWheels = new IntakeWheels();
        servoThickSide = new ShooterFeeder(ShooterConstants.kThickWheelServoPort);
        servoThinSide = new ShooterFeeder(ShooterConstants.kThinWheelServoPort);
        breakBeamSensor = new BreakBeamSensor();

        arm.getShuffleboardTab().add("arm", arm);
        shooterWheels.getShuffleboardTab().add("shooter wheels", shooterWheels);
        shooterRotation.getShuffleboardTab().add("shooter rotation", shooterRotation);
        climber.getShuffleboardTab().add("climber", climber);
        wrist.getShuffleboardTab().add("wrist", wrist);
        intakeWheels.getShuffleboardTab().add("wheels", intakeWheels);
        shooterWheels.getShuffleboardTab().add("servo thick side", servoThickSide);
        // shooterWheels.getShuffleboardTab().add("feeder left", servoLeft);

        // wrist.setDefaultCommand(new RotateWristPID(wrist, 0)); // TODO: what angle
        // does this need to be

        driverController = new CommandXboxController(OIConstants.kDriverController);
        operatorController = new CommandXboxController(OIConstants.kOperatorController);
        configureBindings();
        configureTestCommands();

    }

    public void configureTestCommands() {
        setupArmCommands();
        setupClimberCommands();
        setupIntakeCommands();
        setupShooterCommands();
        // wrist.getShuffleboardTab().add("Deploy Intake", new RotateWristPID(wrist, ));

    }

    private void configureBindings() {
        // Swerve

        final double headingTolerance = Math.toRadians(2.0); // Define a tolerance for heading alignment

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    // Calculate desired heading based on right stick direction
                    Rotation2d desiredHeading = new Rotation2d(driverController.getRightX(),
                            driverController.getRightY());

                    // Calculate the difference between current heading and desired heading
                    Rotation2d currentHeading = drivetrain.getRotation3d().toRotation2d();
                    double headingDifference = desiredHeading.minus(currentHeading).getRadians();

                    return drive.withVelocityX(
                            -driverController.getLeftY() * SwerveConstants.MaxSpeed) // Drive
                                                                                     // forward
                                                                                     // with
                                                                                     // negative
                                                                                     // Y
                            // (forward)
                            .withVelocityY(-driverController.getLeftX()
                                    * SwerveConstants.MaxSpeed)
                            .withRotationalRate(-driverController.getRightX()
                                    * SwerveConstants.MaxAngularRate); // Drive counterclockwise with negative X (left);
                                                                       // Drive left with negative X

                }));

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain
                .applyRequest(() -> point
                        .withModuleDirection(
                                new Rotation2d(-driverController.getLeftY(),
                                        -driverController.getLeftX()))));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);

        driverController.leftTrigger().whileTrue(new IntakeIn(intakeWheels, IntakeConstants.kWheelSpeed));

        // operatorController.povUp()
        // .whileTrue(new RotateShooter(shooterRotation,
        // ShooterConstants.kRotateSpeed));
        // operatorController.povDown()
        // .whileTrue(new RotateShooter(shooterRotation,
        // -ShooterConstants.kRotateSpeed));
    }

    public Command getAutonomousCommand() {
        return runAuto;
    }

    private void setupIntakeCommands() {
        // Intake = Wrist + IntakeWheels
        // INTAKE
        intakeWheels.getShuffleboardTab().add("Run Intake Wheels",
                new IntakeIn(intakeWheels, IntakeConstants.kWheelSpeed));

        wrist.getShuffleboardTab().add("Rotate Intake Simple",
                new RotateWristBasic(wrist,
                        IntakeConstants.kRotateSpeed));

        wrist.getShuffleboardTab().add("Rotate Intake Backwards Simple",
                new RotateWristBasic(wrist,
                        -IntakeConstants.kRotateSpeed));

        wrist.getShuffleboardTab().add("Rotate Intake PID",
                new RotateWristPID(wrist,
                        IntakeConstants.kWristNotePosition));

    }

    private void setupArmCommands() {
        arm.getShuffleboardTab().add("Rotate Arm",
                new RotateArm(arm,
                        ArmConstants.kArmInnerWingSetpoint));
    }

    private void setupShooterCommands() {
        // SHOOTER
        GenericEntry shooterSpeed = shooterWheels.getShuffleboardTab()
                .add("Shooter Velocity", -80)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -100, "max", -30))
                .getEntry();

        GenericEntry shooterAngle = shooterWheels.getShuffleboardTab()
                .add("Shooter Angle", -10)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -70, "max", 0))
                .getEntry();

        shooterWheels.getShuffleboardTab().add("Run Shooter velocity",
                new ShootNoteVelocity(shooterWheels,
                        () -> shooterSpeed.getDouble(ShooterConstants.kShootVelocity)));

        // shooterRotation.getShuffleboardTab().add("Shooter Subwoofer position",
        //         new RotateShooter(shooterRotation,
        //                 ShooterConstants.kArmSubwooferSetpoint));

        // shooterRotation.getShuffleboardTab().add("Shooter 60 inch position",
        //         new RotateShooter(shooterRotation,
        //                 ShooterConstants.kArm60InchSetpoint));

        shooterRotation.getShuffleboardTab().add("Slider Arm Rotation", new RotateShooter(shooterRotation,
                        ShooterConstants.kArm60InchSetpoint));

        shooterRotation.getShuffleboardTab().add("Shooter feed note position TESTING", new RotateShooter(shooterRotation, -70));

        shooterWheels.getShuffleboardTab().add("Rotate Servos", new ParallelCommandGroup(
                new RotateFeeder(servoThickSide, ShooterConstants.kServoThickSideSpeed),
                new RotateFeeder(servoThinSide, ShooterConstants.kServoThinSideSpeed)));

        shooterRotation.getShuffleboardTab().add("Shoot Note Full Command",
                new ParallelCommandGroup(
                        new RotateShooter(shooterRotation, shooterAngle.getDouble(-10)),
                        new ShootNoteVelocity(shooterWheels,
                                () -> shooterSpeed.getDouble(ShooterConstants.kShootVelocity)),
                        new SequentialCommandGroup(
                                new WaitCommand(4),
                                new ParallelCommandGroup(
                                        new ParallelCommandGroup(
                                                new RotateFeeder(servoThickSide, ShooterConstants.kServoThickSideSpeed),
                                                new RotateFeeder(servoThinSide,
                                                        ShooterConstants.kServoThinSideSpeed))))));
    }

    private void setupClimberCommands() {
        climber.getShuffleboardTab().add("Run Climber Simple",
                new ExtendClimber(climber,
                        ClimberConstants.kClimberSpeed));
    }
}
