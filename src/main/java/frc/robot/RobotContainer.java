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
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeUntilNoteIn;
import frc.robot.commands.Intake.RotateWristBasic;
import frc.robot.commands.Intake.RotateWristPID;
import frc.robot.commands.arm.RotateArm;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.controller.ToggleRumble;
import frc.robot.commands.shooter.StopServos;
import frc.robot.commands.shooter.EjectNote;
import frc.robot.commands.shooter.RotateShooter;
import frc.robot.commands.shooter.RotateShooterBasic;
import frc.robot.commands.shooter.ShootNoteMotionMagicVelocity;
import frc.robot.commands.shooter.ShootNoteVelocity;
import frc.robot.commands.shooter.ShootNoteVoltage;
import frc.robot.commands.swerve.AimToAprilTag;
import frc.robot.constants.*;
import frc.robot.constants.DIOConstants.Intake;
import frc.robot.subsystems.Intake.IntakeWheels;
import frc.robot.subsystems.Intake.Wrist;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.sensors.BreakBeamSensorIntake;
import frc.robot.subsystems.sensors.BreakBeamSensorShooter;
import frc.robot.subsystems.shooter.ShooterServos;
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
import frc.robot.utils.LimelightUtils;
import frc.robot.utils.Telemetry;

import java.util.Map;

public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  // My drivetrain

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
  private final ShooterServos servos;
  private final Wrist wrist;
  private final IntakeWheels intakeWheels;
  private final Arm arm;
  private final Climber climber;


//   private final FrontCamera frontCamera;
  private final BackCamera backCamera;
  private final DashBoardManager visionManager;
  private final BreakBeamSensorShooter breakBeamSensorShooter;
  private final BreakBeamSensorIntake breakBeamSensorIntake;

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
    // frontCamera = new FrontCamera();
    backCamera = new BackCamera();
    visionManager = new DashBoardManager();
    servos = new ShooterServos();
    breakBeamSensorShooter = new BreakBeamSensorShooter();
    breakBeamSensorIntake = new BreakBeamSensorIntake();
   

    arm.getShuffleboardTab().add("arm", arm);
    shooterWheels.getShuffleboardTab().add("shooter wheels", shooterWheels);
    shooterRotation.getShuffleboardTab().add("shooter rotation", shooterRotation);
    climber.getShuffleboardTab().add("climber", climber);
    wrist.getShuffleboardTab().add("wrist", wrist);
    intakeWheels.getShuffleboardTab().add("wheels", intakeWheels);
    shooterWheels.getShuffleboardTab().add("servo", servos);

    driverController = new CommandXboxController(OIConstants.kDriverController);
    operatorController = new CommandXboxController(OIConstants.kOperatorController);

    configureDefaultCommands();
    configureDriverController();
    configureOperatorController();

    setupArmCommands();
    setupClimberCommands();
    setupIntakeCommands();
    setupShooterCommands();
    setErrorTriggers();

    Shuffleboard.getTab("swerve").add("Swerve PID", drivetrain.pid);
    // This is done because the
    // CommandSwerveDrive
    // class constructor is really
    // weird
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
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
          Rotation2d.fromDegrees(90)));
    }

    drivetrain
        .registerTelemetry(
            logger::telemeterize);
  }

  private void configureDriverController() {
    // driverController.a()
    // .whileTrue(
    // drivetrain.applyRequest(() -> brake));

    // driverController.b()
    // .whileTrue(drivetrain
    // .applyRequest(() -> point
    // .withModuleDirection(new Rotation2d(
    // -driverController.getLeftY(),
    // -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.start()
        .onTrue(
            drivetrain.runOnce(
                drivetrain::seedFieldRelative));

    driverController.leftTrigger()
        .onTrue(
            new RotateWristPID(wrist, IntakeConstants.kWristNotePosition)
                .andThen(
                    new IntakeIn(intakeWheels, () -> -0.3)))
        .onFalse(new ParallelCommandGroup(
            new RotateWristPID(wrist, IntakeConstants.kWristIdlePosition),
            new IntakeIn(intakeWheels, () -> 0)));

  }

  public void configureOperatorController() {
    // LOAD BUTTON
    operatorController.leftBumper()
        .onTrue(new ParallelDeadlineGroup(
            new RotateWristPID(wrist, IntakeConstants.kWristShooterFeederSetpoint),
            new RotateShooter(shooterRotation, () -> -30))
            .andThen(
                new WaitCommand(0.2)
                    .andThen(
                        new ParallelCommandGroup(
                            new IntakeIn(intakeWheels, () -> 0.4),
                            new EjectNote(servos))
                            .until(breakBeamSensorShooter::getBeamBroken)
                            .andThen(new ParallelDeadlineGroup(
                                new WaitCommand(0.3),
                                new IntakeIn(intakeWheels, () -> 0.4),
                                new EjectNote(servos))))))
        .onFalse(new RotateShooterBasic(shooterRotation, () -> 0));

    // AUTO AIM
    operatorController.rightTrigger()
        .onTrue(new AimToAprilTag(drivetrain, driverController::getLeftX,
            driverController::getLeftY)
            .andThen(new ParallelCommandGroup(
                new ToggleRumble(driverController, 0.5)),
                new ToggleRumble(operatorController, 0.5)))
        .onFalse(new ParallelDeadlineGroup(
            new WaitCommand(0.2),
            drivetrain.applyRequest(() -> brake)));

    operatorController.leftTrigger()
        .onTrue(new ParallelCommandGroup(
            new RotateShooter(shooterRotation,
                () -> LimelightUtils.GetShooterAngle(
                    ExportedVariables.Distance)),
            // new ShootNoteVelocity(shooterWheels, () ->
            // ShooterConstants.kShootVelocity),
            new ShootNoteMotionMagicVelocity(shooterWheels, () -> -80),
            new SequentialCommandGroup(
                new WaitCommand(3),
                new EjectNote(servos))))
        .onFalse(new ParallelCommandGroup(
            new RotateShooterBasic(shooterRotation, () -> 0),
            new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0),
            new StopServos(servos)));

    operatorController.a().onTrue(new RotateShooter(shooterRotation,
        () -> LimelightUtils.GetShooterAngle(ExportedVariables.Distance)))
        .onFalse(new RotateShooterBasic(shooterRotation, () -> 0));

  }

  private void setErrorTriggers() {
    new Trigger(wrist::getFailure).whileTrue(
        new ParallelCommandGroup(
            new ToggleRumble(driverController, 10000),
            new ToggleRumble(operatorController, 10000)));
    new Trigger(arm::getFailure).whileTrue(
        new ParallelCommandGroup(
            new ToggleRumble(driverController, 10000),
            new ToggleRumble(operatorController, 10000)));
    new Trigger(shooterRotation::getFailure).whileTrue(
        new ParallelCommandGroup(
            new ToggleRumble(driverController, 10000),
            new ToggleRumble(operatorController, 10000)));
  }

  public Command getAutonomousCommand() {
    return runAuto;
    // return null;
  }

  private void setupIntakeCommands() {
    // Intake = Wrist + IntakeWheels
    // INTAKE
    intakeWheels.getShuffleboardTab().add("Run Intake Wheels",
        new IntakeIn(intakeWheels, () -> IntakeConstants.kWheelSpeed));

    intakeWheels.getShuffleboardTab().add("Run Intake Wheels Backwards",
        new IntakeIn(intakeWheels, () -> -IntakeConstants.kWheelSpeed));

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

    wrist.getShuffleboardTab().add("Rotate Note Position",
        new RotateWristPID(wrist,
            IntakeConstants.kWristNotePosition));

    wrist.getShuffleboardTab().add("Rotate Idle Position",
        new RotateWristPID(wrist,
            IntakeConstants.kWristIdlePosition));

    wrist.getShuffleboardTab().add("Rotate Shooter Position",
        new RotateWristPID(wrist,
            IntakeConstants.kWristShooterFeederSetpoint));

    wrist.getShuffleboardTab().add("Rotate until note in intake",
        new SequentialCommandGroup(
            new RotateWristPID(wrist, IntakeConstants.kWristNotePosition),
            new IntakeUntilNoteIn(intakeWheels, breakBeamSensorIntake),
            new RotateWristPID(wrist, IntakeConstants.kWristShooterFeederSetpoint)

        ));
  }

  private void setupArmCommands() {
    arm.getShuffleboardTab().add("Rotate Arm",
        new RotateArm(arm,
            () -> ArmConstants.ArmPIDForExternalEncoder.kArmInnerWingSetpoint));
  }

  private void setupShooterCommands() {
    ShuffleboardTab shooterTab = shooterWheels.getShuffleboardTab();

    // SHOOTER
    GenericEntry shooterSpeed = shooterTab
        .add("Shooter Velocity", -80)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -200, "max", -30, "blockIncrement", 2))
        .getEntry();

    shooterTab.add("Run Shooter velocity",
        new ShootNoteVelocity(shooterWheels,
            () -> shooterSpeed.getDouble(ShooterConstants.kShootVelocity)));

    shooterTab.add("Slider Arm Rotation", new RotateShooter(shooterRotation,
        () -> ShooterConstants.RotationPIDForExternalEncoder.kArm60InchSetpoint));

    shooterTab.add("Shooter feed note position TESTING",
        new RotateShooter(shooterRotation, () -> -30));

    shooterTab.add("Spin Servos", new EjectNote(servos));
    shooterTab.add("Stop Servos", new StopServos(servos));

    shooterTab.add("Shoot Note Full Command",
        new ParallelCommandGroup(
            new RotateShooter(shooterRotation,
                () -> LimelightUtils.GetShooterAngle(
                    ExportedVariables.Distance)),
            new ShootNoteVelocity(shooterWheels,
                () -> shooterSpeed.getDouble(
                    ShooterConstants.kShootVelocity)),
            new SequentialCommandGroup(
                new WaitCommand(5),
                new StopServos(servos))));
  }

  private void setupClimberCommands() {
    climber.getShuffleboardTab().add("Run Climber Simple",
        new ExtendClimber(climber,
            () -> ClimberConstants.kClimberSpeed));
  }
}
