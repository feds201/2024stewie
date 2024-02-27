/*
 * *****************************************************************************
 *  * Copyright (c) 2024 FEDS 201. All rights reserved.
 *  *
 *  * This codebase is the property of FEDS 201 Robotics Team.
 *  * Unauthorized copying, reproduction, or distribution of this code, or any
 *  * portion thereof, is strictly prohibited.
 *  *
 *  * This code is provided "as is" and without any express or implied warranties,
 *  * including, without limitation, the implied warranties of merchantability
 *  * and fitness for a particular purpose.
 *  *
 *  * For inquiries or permissions regarding the use of this code, please contact
 *  * feds201@gmail.com
 *  ****************************************************************************
 *
 */

package frc.robot;

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
import frc.robot.commands.arm.RotateArm;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.controller.ToggleRumble;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.RotateWristBasic;
import frc.robot.commands.intake.RotateWristPID;
import frc.robot.commands.shooter.RotateFeeder;
import frc.robot.commands.shooter.RotateShooter;
import frc.robot.commands.shooter.ShootNoteVelocity;
import frc.robot.commands.swerve.AimToAprilTag;
import frc.robot.constants.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.sensors.BreakBeamSensor;
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.shooter.ShooterWheels;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.vision_sys.LockTag.Joystick;
import frc.robot.subsystems.vision_sys.LockTag.RotationSource;
import frc.robot.subsystems.vision_sys.VisionVariables.ExportedVariables;
import frc.robot.subsystems.vision_sys.camera.BackCamera;
import frc.robot.subsystems.vision_sys.camera.FrontCamera;
import frc.robot.subsystems.vision_sys.utils.DashBoardManager;
import frc.robot.utils.LLDistanceToShooterAngle;
import frc.robot.utils.Telemetry;

import java.util.Map;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final ShooterWheels shooterWheels;
  private final ShooterRotation shooterRotation;
  private final ShooterFeeder servoThinSide; // These designations refer to the wheels on the intake being more on
                                             // one
                                             // side (thick) and less on the other (thin)
  private final ShooterFeeder servoThickSide;
  private final Wrist wrist;
  private final IntakeWheels intakeWheels;
  private final Arm arm;
  private final Climber climber;

  private final FrontCamera frontCamera;
  private final BackCamera backCamera;
  private final DashBoardManager visionManager;
  private final BreakBeamSensor breakBeamSensor;
  private final CommandXboxController driverController;

  private final CommandXboxController operatorController;
  private RotationSource TabLock = new Joystick();

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
    visionManager = new DashBoardManager();
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

    driverController = new CommandXboxController(OIConstants.kDriverController);
    operatorController = new CommandXboxController(OIConstants.kOperatorController);

    configureDefaultCommands();
    configureDriverStationCommands();
    setupArmCommands();
    setupClimberCommands();
    setupIntakeCommands();
    setupShooterCommands();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          return drive
              .withVelocityX(-driverController.getLeftY()
                  * SwerveConstants.MaxSpeed)
              .withVelocityY(-driverController.getLeftX()
                  * SwerveConstants.MaxSpeed)
              .withRotationalRate(-driverController.getRightX() *
                  SwerveConstants.MaxAngularRate);
        }));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain
        .registerTelemetry(
            logger::telemeterize);
  }

  private void configureDriverStationCommands() {
    driverController.a()
        .whileTrue(
            drivetrain.applyRequest(() -> brake));

    driverController.b()
        .whileTrue(drivetrain
            .applyRequest(() -> point
                .withModuleDirection(new Rotation2d(
                    -driverController.getLeftY(),
                    -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.start()
        .onTrue(
            drivetrain.runOnce(
                drivetrain::seedFieldRelative));

    // LOAD BUTTON
    operatorController.leftBumper().whileTrue(new RotateShooter(shooterRotation, -70));

    // AUTO AIM
    operatorController.rightTrigger()
        .onTrue(new AimToAprilTag(drivetrain, driverController::getLeftX, driverController::getLeftY)
        .andThen(new ToggleRumble(driverController, 0.5)));

//    operatorController.leftTrigger()
//    .onTrue(new ParallelCommandGroup(
//            new RotateShooter(shooterRotation, LLDistanceToShooterAngle.LLDistanceToShooterAngle(ExportedVariables.Distance)),
//            new ShootNoteVelocity(shooterWheels, () -> -ShooterConstants.kShootVelocity),
//            new SequentialCommandGroup(
//                new WaitCommand(5),
//                new ParallelCommandGroup(
//                    new ParallelCommandGroup((
//                        new RotateFeeder(servoThickSide, () -> ShooterConstants.kServoThickSideSpeed),
//                        new RotateFeeder(servoThinSide, () -> ShooterConstants.kServoThinSideSpeed))))))
//    .onFalse(new ParallelCommandGroup(
//            new RotateShooter(shooterRotation, 0),
//            new ShootNoteVelocity(shooterWheels, () -> 0),
//            new RotateFeeder(servoThickSide, () -> 0),
//            new RotateFeeder(servoThinSide, () -> 0)
//            ));
    operatorController.leftTrigger()

            .onTrue(new ParallelCommandGroup(
                    new RotateShooter(shooterRotation, LLDistanceToShooterAngle.LLDistanceToShooterAngle(ExportedVariables.Distance)),
                    new ShootNoteVelocity(shooterWheels, () -> -ShooterConstants.kShootVelocity),
                    new SequentialCommandGroup(
                            new WaitCommand(5),
                            new ParallelCommandGroup(
                                    new ParallelCommandGroup(
                                            new RotateFeeder(servoThickSide, () -> ShooterConstants.kServoThickSideSpeed),
                                            new RotateFeeder(servoThinSide, () -> ShooterConstants.kServoThinSideSpeed)
                                    )
                            )
                    )
            )
            )
            .onFalse(new ParallelCommandGroup(
                    new RotateShooter(shooterRotation, 0),
                    new ShootNoteVelocity(shooterWheels, () -> 0),
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ParallelCommandGroup(
                                            new RotateFeeder(servoThickSide, () -> 0),
                                            new RotateFeeder(servoThinSide, () -> 0)
                                    )
                            )
                    )
            ));


  }

  public Command getAutonomousCommand() {
    return runAuto;
  }

  private void setupIntakeCommands() {
    // Intake = Wrist + IntakeWheels
    // INTAKE
    intakeWheels.getShuffleboardTab().add("Run Intake Wheels",
        new IntakeIn(intakeWheels, () -> IntakeConstants.kWheelSpeed));

    GenericEntry wristSpeed = wrist.getShuffleboardTab()
        .add("Wrist Speed", IntakeConstants.kRotateSpeed)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 0.3, "blockIncrement", 0.005))
        .getEntry();

    wrist.getShuffleboardTab().add("Rotate Intake Simple",
        new RotateWristBasic(wrist,
            () -> wristSpeed.getDouble(IntakeConstants.kRotateSpeed)));

    wrist.getShuffleboardTab().add("Rotate Intake Backwards Simple",
        new RotateWristBasic(wrist,
            () -> -wristSpeed.getDouble(IntakeConstants.kRotateSpeed)));

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
        .withProperties(Map.of("min", -100, "max", -30, "blockIncrement", 1))
        .getEntry();

    GenericEntry shooterAngle = shooterWheels.getShuffleboardTab()
        .add("Shooter Angle", -10)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -70, "max", 0, "blockIncrement", 1))
        .getEntry();

    shooterWheels.getShuffleboardTab().add("Run Shooter velocity",
        new ShootNoteVelocity(shooterWheels,
            () -> shooterSpeed.getDouble(ShooterConstants.kShootVelocity)));

    // shooterRotation.getShuffleboardTab().add("Shooter Subwoofer position",
    // new RotateShooter(shooterRotation,
    // ShooterConstants.kArmSubwooferSetpoint));

    // shooterRotation.getShuffleboardTab().add("Shooter 60 inch position",
    // new RotateShooter(shooterRotation,
    // ShooterConstants.kArm60InchSetpoint));

    shooterRotation.getShuffleboardTab().add("Slider Arm Rotation", new RotateShooter(shooterRotation,
        ShooterConstants.kArm60InchSetpoint));

    shooterRotation.getShuffleboardTab().add("Shooter feed note position TESTING",
        new RotateShooter(shooterRotation, -70));

    shooterWheels.getShuffleboardTab().add("Rotate Servos", new ParallelCommandGroup(
        new RotateFeeder(servoThickSide, () -> ShooterConstants.kServoThickSideSpeed),
        new RotateFeeder(servoThinSide, () -> ShooterConstants.kServoThinSideSpeed)));

    shooterRotation.getShuffleboardTab().add("Shoot Note Full Command",
        new ParallelCommandGroup(
            new RotateShooter(shooterRotation, LLDistanceToShooterAngle.LLDistanceToShooterAngle(ExportedVariables.Distance)),
            new ShootNoteVelocity(shooterWheels,
                () -> shooterSpeed.getDouble(
                    ShooterConstants.kShootVelocity)),
            new SequentialCommandGroup(
                new WaitCommand(5),
                new ParallelCommandGroup(
                    new ParallelCommandGroup(
                        new RotateFeeder(
                            servoThickSide,
                            () -> ShooterConstants.kServoThickSideSpeed),
                        new RotateFeeder(
                            servoThinSide,
                            () -> ShooterConstants.kServoThinSideSpeed))))));
  }

  private void setupClimberCommands() {
    climber.getShuffleboardTab().add("Run Climber Simple",
        new ExtendClimber(climber,
            () -> ClimberConstants.kClimberSpeed));
  }
}
