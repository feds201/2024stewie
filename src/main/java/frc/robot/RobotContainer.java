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
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Intake.RotateWristToPosition;
import frc.robot.commands.Intake.RunIntakeWheels;
import frc.robot.commands.arm.RotateArmToPosition;
import frc.robot.commands.arm.RotateArmManual;
import frc.robot.commands.autons.DriveForwardForTime;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.compound.*;
import frc.robot.commands.controller.ToggleRumble;
import frc.robot.commands.leds.SetLEDColor;
import frc.robot.commands.shooter.StopServos;
import frc.robot.commands.swerve.AimToAprilTag;
import frc.robot.commands.shooter.EjectNote;
import frc.robot.commands.shooter.RotateShooterToPosition;
import frc.robot.commands.shooter.RotateShooterBasic;
import frc.robot.commands.shooter.ShootNoteMotionMagicVelocity;
import frc.robot.commands.swerve.SetFieldRelative;
import frc.robot.constants.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.Intake.IntakeWheels;
import frc.robot.subsystems.Intake.Wrist;
import frc.robot.subsystems.sensors.BreakBeamSensorIntake;
import frc.robot.subsystems.sensors.BreakBeamSensorShooter;
import frc.robot.subsystems.shooter.ShooterServos;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.shooter.ShooterWheels;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.vision_sys.VisionVariables.ExportedVariables;
import frc.robot.subsystems.vision_sys.camera.BackCamera;
import frc.robot.subsystems.vision_sys.utils.DashBoardManager;
import frc.robot.utils.Telemetry;

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

        private final ShooterWheels shooterWheels;
        private final ShooterRotation shooterRotation;
        private final ShooterServos servos;
        private final Wrist wrist;
        private final IntakeWheels intakeWheels;
        private final Arm arm;
        private final Climber climber;
        public final Leds leds;

        // private final FrontCamera frontCamera;
        private final BackCamera backCamera;
        private final DashBoardManager visionManager;
        private final BreakBeamSensorShooter breakBeamSensorShooter;
        private final BreakBeamSensorIntake breakBeamSensorIntake;

        public final CommandXboxController driverController;
        public final CommandXboxController operatorController;

        SendableChooser<Command> autonChooser = new SendableChooser<>();

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
                leds = new Leds();

                arm.getShuffleboardTab().add("arm", arm);
                shooterWheels.getShuffleboardTab().add("shooter wheels", shooterWheels);
                shooterRotation.getShuffleboardTab().add("shooter rotation", shooterRotation);
                climber.getShuffleboardTab().add("climber", climber);
                wrist.getShuffleboardTab().add("wrist", wrist);
                intakeWheels.getShuffleboardTab().add("wheels", intakeWheels);
                shooterWheels.getShuffleboardTab().add("servo", servos);

                driverController = new CommandXboxController(OIConstants.kDriverController);
                operatorController = new CommandXboxController(OIConstants.kOperatorController);

                

                registerAllAutoCommands();
                configureDefaultCommands();
                configureDriverController();
                configureOperatorController();

                setupArmCommands();;
                setupClimberCommands();
                setupIntakeCommands();
                setupShooterCommands();
                setupErrorTriggers();
                setupAutonCommands();

        }
<<<<<<< HEAD
        
        drivetrain
            .registerTelemetry(
                logger::telemeterize);
        
        arm.setDefaultCommand(
            new RotateArmManual(
                arm,
                () -> -operatorController.getLeftY()));
        
        new Trigger(() -> arm.getArmAngle() > ArmConstants.kArmClimbLimit)
            .onTrue(new InstantCommand(() -> leds.setLedColor(Leds.LedColors.YELLOW)))
            .onFalse(new InstantCommand(() -> leds.setLedColor(Leds.getAllianceColor())));
        
        shooterWheels.setDefaultCommand(new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0));
    }
    
    private void configureDriverController() {
        // reset the field-centric heading on left bumper press
        driverController.start()
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));
        
        driverController.leftTrigger()
            .onTrue(
                new DeployIntake(wrist, intakeWheels, shooterRotation, intakeIRSensor, leds, driverController, operatorController)
            )
            .onFalse(
                new ParallelCommandGroup(new ResetIntake(wrist, intakeWheels),
                    new ToggleRumble(driverController, 0),
                    new ToggleRumble(operatorController, 0),
                    new SequentialCommandGroup(new WaitCommand(1.5), new SetLEDColor(leds, Leds.LedColors.WHITE))
                        .onlyIf(intakeIRSensor::getBeamBroken)));
        
        driverController.rightTrigger()
            .onTrue(new SpitOutNote(wrist, intakeWheels))
            .onFalse(new ResetIntake(wrist, intakeWheels));
        
        driverController.rightBumper()
            .onTrue(new RunIntakeWheels(intakeWheels, () -> kIntakeNoteWheelSpeed))
            .onFalse(new RunIntakeWheels(intakeWheels, () -> 0));
        
        driverController.a()
            .onTrue(new RotateShooterToPosition(shooterRotation,
                () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint));
        
        driverController.x()
            .onTrue(
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new ParallelDeadlineGroup(
                            new WaitCommand(2),
                            new RotateShooterToPosition(shooterRotation,
                                () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterHorizontal)),
                        new ShootNoteMotionMagicVelocity(
                            shooterWheels,
                            () -> LimelightUtils.GetSpeedTop(VisionVariables.ExportedVariables.Distance),
                            () -> LimelightUtils.GetSpeedBottom(VisionVariables.ExportedVariables.Distance)
                        ),
                        new SequentialCommandGroup(
                            new WaitCommand(0.7),
                            new EjectNote(servos),
                            new SetLEDColor(leds, Leds.getAllianceColor()))),
                    new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0),
                    new RotateShooterToPosition(shooterRotation,
                        () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint)))
            .onFalse(
                new ParallelCommandGroup(
                    new RotateShooterToPosition(shooterRotation,
                        () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint),
                    new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0)));
        
        driverController.leftBumper()
            .onTrue(new InstantCommand(() -> swerveSpeedMultiplier = 0.2))
            .onFalse(new InstantCommand(() -> swerveSpeedMultiplier = 1));
    }
    
    public void configureOperatorController() {
        // LOAD BUTTON
        operatorController.leftBumper()
            .onTrue(new ParallelCommandGroup(
                new RotateArmToPosition(arm, () -> 0),
                new AlignShooterAndIntake(shooterRotation, wrist, intakeWheels,
                    servos, shooterIRSensor, leds)));
        
        operatorController.rightTrigger()
            .onTrue(
                new ParallelCommandGroup(
                    new AimToAprilTag(drivetrain,
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getLeftY(),
                        () -> VisionVariables.ExportedVariables.Distance)
                        .andThen(
                            new ParallelCommandGroup(
                                new SetLEDColor(leds, Leds.LedColors.VIOLET),
                                new ToggleRumble(driverController, 0.3),
                                new ToggleRumble(operatorController, 0.3))
                        ),
                    new RotateWristToPositionInfinite(wrist, IntakeConstants.WristPID.kWristOutOfTheWay)
                )
            )
            .onFalse(
                new ParallelDeadlineGroup(
                    new WaitCommand(0.2),
                    drivetrain.applyRequest(() -> brake),
                    new RotateWristToPosition(wrist, IntakeConstants.WristPID.kWristShooterFeederSetpoint)
                )
            );
        
        
        operatorController.leftTrigger()
            .onTrue(
                new ParallelCommandGroup(
                    new RotateWristToPositionInfinite(wrist, IntakeConstants.WristPID.kWristOutOfTheWay),
                    new ShootFromHandoff(shooterRotation, shooterWheels, servos, leds, () -> VisionVariables.ExportedVariables.Distance, shooterIRSensor))
                    .andThen(
                        new ParallelCommandGroup(
                            new ToggleRumble(driverController, 0.3),
                            new ToggleRumble(operatorController,
                                0.3)))
            
            )
            .onFalse(new ParallelCommandGroup(
                new SetLEDColor(leds, Leds.getAllianceColor()),
                new RotateShooterToPosition(shooterRotation,
                    () -> ShooterConstants.RotationPIDForExternalEncoder.kShooterRotationFeederSetpoint),
                new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0),
                new ResetIntake(wrist, intakeWheels),
                new ToggleRumble(driverController, 0),
                new ToggleRumble(operatorController, 0)
            ));
        
        operatorController.a()
            .onTrue(new PlaceInAmp(wrist, intakeWheels, arm, leds, shooterRotation)
                .andThen(
                    new ParallelCommandGroup(
                        new SetLEDColor(leds,
                            leds.getLedColor()),
                        new ToggleRumble(driverController, 0.3),
                        new ToggleRumble(operatorController, 0.3))))
            .onFalse(new ParallelCommandGroup(
                new RotateWristToPosition(wrist,
                    IntakeConstants.WristPID.kWristIdlePosition),
                new RotateArmToPosition(arm, () -> 0),
                new RunIntakeWheels(intakeWheels, () -> 0)));
        operatorController.x()
            .onTrue(new RotateWristToPosition(wrist, IntakeConstants.WristPID.kWristShooterFeederSetpoint));
        // new Trigger(breakBeamSensorShooter::getBeamBroken).onTrue(new
        // SetLEDColor(leds, Leds.LedColors.ORANGE));
//        operatorController.povUp()
//            .onTrue()
    
    }
    
    private void setupErrorTriggers() {
        // There should be some feedback for an "failure mode" but rumbling the
        // controller continuously was obnoxious lol.
    }
    
    public Command getAutonomousCommand() {
        return autonChooser.getSelected(); // runAuto;
        // return null;
    }
    
    private void setupIntakeCommands() {
        intakeWheels.getShuffleboardTab().add("Intake Wheels",
            new RunIntakeWheels(intakeWheels, () -> kIntakeNoteWheelSpeed));
        
        intakeWheels.getShuffleboardTab().add("Reverse Intake Wheels",
            new RunIntakeWheels(intakeWheels, () -> -kIntakeNoteWheelSpeed));
        
        wrist.getShuffleboardTab().add("Note Position",
            new RotateWristToPosition(wrist,
                IntakeConstants.WristPID.kWristNotePosition));
        
        wrist.getShuffleboardTab().add("Idle Position",
            new RotateWristToPosition(wrist,
                IntakeConstants.WristPID.kWristIdlePosition));
        
        wrist.getShuffleboardTab().add("Shooter Position",
            new RotateWristToPosition(wrist,
                IntakeConstants.WristPID.kWristShooterFeederSetpoint));
        
        wrist.getShuffleboardTab().add("Rotate until note in intake",
            new SequentialCommandGroup(
                new RotateWristToPosition(wrist,
                    IntakeConstants.WristPID.kWristNotePosition),
                new IntakeUntilNoteIn(intakeWheels, intakeIRSensor, leds, driverController, operatorController),
                new RotateWristToPosition(wrist,
                    IntakeConstants.WristPID.kWristShooterFeederSetpoint)
            ));
    }
    
    private void setupArmCommands() {
        arm.getShuffleboardTab().add("Rotate Arm",
            new RotateArmToPosition(arm,
                () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint));
    }
    
    private void setupShooterCommands() {
        ShuffleboardTab shooterTab = shooterWheels.getShuffleboardTab();
        
        shooterTab.add("100 RPS Shoot",
            new ShootNoteMotionMagicVelocity(shooterWheels,
                () -> ShooterConstants.kShootVelocity, () -> ShooterConstants.kShootVelocity));
        
        shooterTab.add("0 RPS Shoot",
            new ShootNoteMotionMagicVelocity(shooterWheels,
                () -> 0, () -> 0));
        
        shooterTab.add("-15 Deg Rotate", new RotateShooterToPosition(shooterRotation,
            () -> ShooterConstants.RotationPIDForExternalEncoder.kArm60InchSetpoint));
        
        shooterTab.add("-30 Deg Rotate",
            new RotateShooterToPosition(shooterRotation, () -> -30));
        
        shooterTab.add("Spin Servos", new EjectNote(servos));
        
        shooterTab.add("Stop Servos", new StopServos(servos));
        
        shooterTab.add("Shoot Note Full Command",
            new ShootNoteAtSpeakerOnly(shooterRotation, shooterWheels, servos, leds, () -> VisionVariables.ExportedVariables.Distance, shooterIRSensor));
    }
    
    private void setupClimberCommands() {
        climber.getShuffleboardTab().add("Climb Simple",
            new ExtendClimber(climber,
                () -> ClimberConstants.kClimberSpeed));
    }
    
    private void printCurrentStickValues() {
        SmartDashboard.putNumber("left y",
            -driverController.getLeftY()
                * SwerveConstants.MaxSpeed * swerveSpeedMultiplier);
        SmartDashboard.putNumber("left x",
            -driverController.getLeftX()
                * SwerveConstants.MaxSpeed * swerveSpeedMultiplier);
        SmartDashboard.putNumber("right x",
            -driverController.getRightX() *
                SwerveConstants.MaxAngularRate * swerveSpeedMultiplier);
    }
=======

        private void registerAllAutoCommands() {
                NamedCommands.registerCommand("ShootNoteAtSpeakerOnly",
                                new ShootNoteAtSpeakerOnly(shooterRotation, shooterWheels, servos,
                                                breakBeamSensorShooter));
                NamedCommands.registerCommand("DeployIntake",
                                new DeployIntake(wrist, intakeWheels, shooterRotation, breakBeamSensorIntake, leds));
                NamedCommands.registerCommand("AlignShooterAndIntake",
                                new AlignShooterAndIntake(shooterRotation, wrist, intakeWheels, servos,
                                                breakBeamSensorShooter, leds));
                NamedCommands.registerCommand("ShootFromHandoff", //
                                new ShootFromHandoff(wrist, shooterRotation, shooterWheels, servos,
                                                breakBeamSensorShooter));
                NamedCommands.registerCommand("StopShooterWheelsPls",
                                new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0));
                NamedCommands.registerCommand("AimToAprilTag", new AimToAprilTag(drivetrain, driverController::getLeftX,
                                driverController::getLeftY, () -> ExportedVariables.Distance));
                NamedCommands.registerCommand("ArmDown",  new RotateArmToPosition(arm,() -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint));
                // NamedCommands.registerCommand("FeedNoteToShooter", new
                // AlignShooterAndIntake(shooterRotation, wrist, intakeWheels,
                // servos, breakBeamSensorShooter, leds));
                NamedCommands.registerCommand("DropIntake", new DropIntake(wrist, arm));
                NamedCommands.registerCommand("SetFieldRelative", new SetFieldRelative(drivetrain));
                NamedCommands.registerCommand("AlignWithNote", new DriveForwardForTime(drivetrain, 2));
        }

        private void setupAutonCommands() {

                autonChooser.setDefaultOption("Rotate Arm To Zero Pos",
                                new RotateArmToPosition(arm,
                                                () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint));

                autonChooser.addOption("Shoot In Front of Speaker",
                                new ParallelCommandGroup(
                                                new RotateArmToPosition(arm,
                                                                () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(0.5),
                                                                new ShootNoteAtSpeakerOnly(shooterRotation,
                                                                                shooterWheels, servos,
                                                                                breakBeamSensorShooter))));

                autonChooser.addOption("Step Back Fade Away",
                                new ParallelCommandGroup(
                                                new DriveForwardForTime(drivetrain, 2),
                                                new RotateArmToPosition(arm,
                                                                () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(6),
                                                                new ShootNoteAtSpeakerOnly(shooterRotation,
                                                                                shooterWheels, servos,
                                                                                breakBeamSensorShooter),
                                                                new WaitCommand(2),
                                                                new ParallelCommandGroup(
                                                                                new ShootNoteMotionMagicVelocity(
                                                                                                shooterWheels,
                                                                                                () -> 0)))));

                autonChooser.addOption("TestCode",
                                drivetrain.getAutoPath("test"));

                autonChooser.addOption("Step Back Fade Away and Run",
                                new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                                new DriveForwardForTime(drivetrain, 2),
                                                                new RotateArmToPosition(arm,
                                                                                () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint),
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(3),
                                                                                new ShootNoteAtSpeakerOnly(
                                                                                                shooterRotation,
                                                                                                shooterWheels, servos,
                                                                                                breakBeamSensorShooter))),
                                                new DriveForwardForTime(drivetrain, 5)));

                autonChooser.addOption("Shoot and Scram",
                                new SequentialCommandGroup(
                                                new ParallelDeadlineGroup(
                                                                new WaitCommand(2),
                                                                new RotateArmToPosition(arm,
                                                                                () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint),
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(0.7),
                                                                                new ShootFromHandoff(wrist,shooterRotation,shooterWheels,servos,breakBeamSensorShooter))),
//                                                                                new Shoot(
//                                                                                                shooterRotation,
//                                                                                                shooterWheels, servos,
//                                                                                                breakBeamSensorShooter)


                                                new ParallelCommandGroup(
                                                                new RotateShooterBasic(shooterRotation, () -> 0),
                                                                new DriveForwardForTime(drivetrain, 6))));

                autonChooser.addOption("Red-2Note-CenterStart",
                        drivetrain.getAutoPath("Red-2Note-CenterStart"));
                    ;
                autonChooser.addOption("Blue-2Note-CenterStart",
                                drivetrain.getAutoPath("Blue-2Note-CenterStart"));

                autonChooser.addOption("Place Arm Down and 2 note move then shoot", new ParallelCommandGroup(
                                new RotateArmToPosition(arm,
                                                () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint),
                                new SequentialCommandGroup(
                                                new WaitCommand(0.8),
                                                drivetrain.getAutoPath("2 Note Move Then Shoot"))));

                autonChooser.addOption("Place Arm Down and 2 note", new ParallelCommandGroup(
                                new RotateArmToPosition(arm,
                                                () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint),
                                new SequentialCommandGroup(
                                                new WaitCommand(0.8),
                                                drivetrain.getAutoPath("2 Note Shoot Then Move"))));

                autonChooser.addOption("Place Arm Down and 2 note source side", new ParallelCommandGroup(
                                new RotateArmToPosition(arm,
                                                () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint),
                                new SequentialCommandGroup(
                                                new WaitCommand(0.8),
                                                drivetrain.getAutoPath("2 Note Move Then Shoot Source Side"))));

                // autonChooser.addOption("Aim and Shoot Auton", new ParallelCommandGroup(
                // new RotateArm(arm, () ->
                // ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint),
                // new SequentialCommandGroup(
                // new WaitCommand(6),
                // new AimToAprilTag(drivetrain, driverController::getLeftX,
                // driverController::getLeftY),
                // new ShootNoteAtSpeakerOnly(shooterRotation, shooterWheels, servos))));

                Shuffleboard.getTab("autons").add(autonChooser);
        }

        private void configureDefaultCommands() {
                drivetrain.setDefaultCommand(new ParallelCommandGroup(
                                drivetrain.applyRequest(() -> {
                                        return drive
                                                        .withVelocityX(-driverController.getLeftY()
                                                                        * SwerveConstants.MaxSpeed)
                                                        .withVelocityY(-driverController.getLeftX()
                                                                        * SwerveConstants.MaxSpeed)
                                                        .withRotationalRate(-driverController.getRightX() *
                                                                        SwerveConstants.MaxAngularRate);
                                }), new RepeatCommand(
                                                new InstantCommand(
                                                                () -> {
                                                                        SmartDashboard.putNumber("left y",
                                                                                        -driverController.getLeftY()
                                                                                                        * SwerveConstants.MaxSpeed);
                                                                        SmartDashboard.putNumber("left x",
                                                                                        -driverController.getLeftX()
                                                                                                        * SwerveConstants.MaxSpeed);
                                                                        SmartDashboard.putNumber("right x",
                                                                                        -driverController.getRightX() *
                                                                                                        SwerveConstants.MaxAngularRate);
                                                                }))));

                if (Utils.isSimulation()) {
                        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
                                        Rotation2d.fromDegrees(90)));
                }

                drivetrain
                                .registerTelemetry(
                                                logger::telemeterize);

                arm.setDefaultCommand(new RotateArmManual(arm, () -> -operatorController.getLeftY()));

                shooterWheels.setDefaultCommand(new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0));
        }

        private void configureDriverController() {
                // reset the field-centric heading on left bumper press
                driverController.start()
                                .onTrue(
                                                drivetrain.runOnce(
                                                                drivetrain::seedFieldRelative));

                driverController.leftTrigger()
                                .onTrue(
                                                new SequentialCommandGroup(
                                                                new DeployIntake(wrist, intakeWheels, shooterRotation,
                                                                                breakBeamSensorIntake, leds),
                                                                new ParallelCommandGroup(
                                                                                new SetLEDColor(leds,
                                                                                                Leds.LedColors.YELLOW),
                                                                                new ToggleRumble(driverController, 0.3),
                                                                                new ToggleRumble(operatorController,
                                                                                                0.3))))
                                .onFalse(new ResetIntake(wrist, intakeWheels));

                driverController.rightTrigger()
                                .onTrue(new SpitOutNote(wrist, intakeWheels))
                                .onFalse(new ResetIntake(wrist, intakeWheels));

        }

        public void configureOperatorController() {
                // LOAD BUTTON
                operatorController.leftBumper()
                                .onTrue(new ParallelCommandGroup(
                                                new RotateArmToPosition(arm, () -> 0),
                                                new AlignShooterAndIntake(shooterRotation, wrist, intakeWheels,
                                                                servos, breakBeamSensorShooter, leds)));
//                operatorController.rightBumper()
//                                .onTrue(new ParallelCommandGroup(
//                                                new ShootAcrossField(shooterRotation, shooterWheels, servos, breakBeamSensorShooter)
//                                ))
//                                .onFalse(new ParallelCommandGroup(
//                                        new RotateShooterBasic(shooterRotation, () -> 0),
//                                        new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0),
//                                        new ResetIntake(wrist, intakeWheels)));
                // .onFalse(new ParallelCommandGroup(
                // // new RotateShooter(shooterRotation, () -> -5),
                // new ResetIntake(wrist, intakeWheels)));

                // AUTO AIM
                operatorController.rightTrigger()
                                .onTrue(new AimToAprilTag(drivetrain, driverController::getLeftX,
                                                driverController::getLeftY, () -> ExportedVariables.Distance)
                                                .andThen(
                                                                new ParallelCommandGroup(
                                                                                new SetLEDColor(leds,
                                                                                                Leds.LedColors.VIOLET),
                                                                                new ToggleRumble(driverController, 0.3),
                                                                                new ToggleRumble(operatorController,
                                                                                                0.3))))
                                .onFalse(new ParallelDeadlineGroup(
                                                new WaitCommand(0.2),
                                                drivetrain.applyRequest(() -> brake)));

                operatorController.leftTrigger()
                                .onTrue(new SequentialCommandGroup(
                                                new SetLEDColor(leds, leds.getAllianceColor()),
                                                new ShootFromHandoff(wrist, shooterRotation, shooterWheels, servos,
                                                                breakBeamSensorShooter))
                                                .andThen(
                                                                new ParallelCommandGroup(
                                                                                new SetLEDColor(leds, leds
                                                                                                .getAllianceColor()),
                                                                                new ToggleRumble(driverController, 0.3),
                                                                                new ToggleRumble(operatorController,
                                                                                                0.3))))
                                .onFalse(new ParallelCommandGroup(
                                                new RotateShooterBasic(shooterRotation, () -> 0),
                                                new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0),
                                                new ResetIntake(wrist, intakeWheels)));

                operatorController.b()
                                .onTrue(new SpitOutNote(wrist, intakeWheels)
                                                .andThen(
                                                                new ParallelCommandGroup(
                                                                                new SetLEDColor(leds, leds
                                                                                                .getAllianceColor()),
                                                                                new ToggleRumble(driverController, 0.3),
                                                                                new ToggleRumble(operatorController,
                                                                                                0.3))))
                                .onFalse(new ResetIntake(wrist, intakeWheels));
                operatorController.povUp()
                                .onTrue(new RotateArmToPosition(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kAmpPosition));
                operatorController.povDown()
                                .onTrue(new RotateArmToPosition(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint));
                operatorController.a()
                                .onTrue(new PlaceInAmp(wrist, intakeWheels, arm, leds)
                                                .andThen(
                                                                new ParallelCommandGroup(
                                                                                new SetLEDColor(leds,
                                                                                                leds.getLedColor()),
                                                                                new ToggleRumble(driverController, 0.3),
                                                                                new ToggleRumble(operatorController,
                                                                                                0.3))))
                                .onFalse(new ParallelCommandGroup(
                                                new RotateWristToPosition(wrist,
                                                                IntakeConstants.WristPID.kWristIdlePosition),
                                                new RotateArmToPosition(arm, () -> 0),
                                                new RunIntakeWheels(intakeWheels, () -> 0)));
                operatorController.x()
                        .onTrue(new RotateWristToPosition(wrist, IntakeConstants.WristPID.kWristShooterFeederSetpoint));
                // new Trigger(breakBeamSensorShooter::getBeamBroken).onTrue(new
                // SetLEDColor(leds, Leds.LedColors.ORANGE));

        }

        private void setupErrorTriggers() {
                // There should be some feedback for an "failure mode" but rumbling the
                // controller continuously was obnoxious lol.
        }

        public Command getAutonomousCommand() {
                return autonChooser.getSelected(); // runAuto;
                // return null;
        }

        private void setupIntakeCommands() {
                // Intake = Wrist + IntakeWheels
                // INTAKE
                intakeWheels.getShuffleboardTab().add("Run Intake Wheels",
                                new RunIntakeWheels(intakeWheels, () -> IntakeConstants.kIntakeNoteWheelSpeed));

                intakeWheels.getShuffleboardTab().add("Run Intake Wheels Backwards",
                                new RunIntakeWheels(intakeWheels, () -> -IntakeConstants.kIntakeNoteWheelSpeed));

                // GenericEntry wristSpeed = wrist.getShuffleboardTab()
                // .add("Wrist Speed", IntakeConstants.kRotateSpeed)
                // .withWidget(BuiltInWidgets.kNumberSlider)
                // .withProperties(Map.of("min", 0, "max", 0.8, "blockIncrement", 0.005))
                // .getEntry();

                // wrist.getShuffleboardTab().add("Rotate Intake Simple",
                // new RotateWristBasic(wrist,
                // () -> wristSpeed.getDouble(IntakeConstants.kRotateSpeed)));

                // wrist.getShuffleboardTab().add("Rotate Intake Backwards Simple",
                // new RotateWristBasic(wrist,
                // () -> -wristSpeed.getDouble(IntakeConstants.kRotateSpeed)));

                wrist.getShuffleboardTab().add("Rotate Note Position",
                                new RotateWristToPosition(wrist,
                                                IntakeConstants.WristPID.kWristNotePosition));

                wrist.getShuffleboardTab().add("Rotate Idle Position",
                                new RotateWristToPosition(wrist,
                                                IntakeConstants.WristPID.kWristIdlePosition));

                wrist.getShuffleboardTab().add("Rotate Shooter Position",
                                new RotateWristToPosition(wrist,
                                                IntakeConstants.WristPID.kWristShooterFeederSetpoint));

                wrist.getShuffleboardTab().add("Rotate until note in Intake",
                                new SequentialCommandGroup(
                                                new RotateWristToPosition(wrist,
                                                                IntakeConstants.WristPID.kWristNotePosition),
                                                new IntakeUntilNoteIn(intakeWheels, breakBeamSensorIntake, leds),
                                                new RotateWristToPosition(wrist,
                                                                IntakeConstants.WristPID.kWristShooterFeederSetpoint)

                                ));
        }

        private void setupArmCommands() {
                arm.getShuffleboardTab().add("Rotate Arm",
                                new RotateArmToPosition(arm,
                                                () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint));
        }

        private void setupShooterCommands() {
                ShuffleboardTab shooterTab = shooterWheels.getShuffleboardTab();

                // SHOOTER

                shooterTab.add("Run Shooter velocity",
                                new ShootNoteMotionMagicVelocity(shooterWheels,
                                                () -> -80));

                shooterTab.add("Slider Arm Rotation", new RotateShooterToPosition(shooterRotation,
                                () -> ShooterConstants.RotationPIDForExternalEncoder.kArm60InchSetpoint));

                shooterTab.add("Shooter feed note position TESTING",
                                new RotateShooterToPosition(shooterRotation, () -> -30));

                shooterTab.add("Spin Servos", new EjectNote(servos));
                shooterTab.add("Stop Servos", new StopServos(servos));

                shooterTab.add("Shoot Note Full Command",
                                new ShootNoteAtSpeakerOnly(shooterRotation, shooterWheels, servos,
                                                breakBeamSensorShooter));

        }

        private void setupClimberCommands() {
                climber.getShuffleboardTab().add("Run Climber Simple",
                                new ExtendClimber(climber,
                                                () -> ClimberConstants.kClimberSpeed));
        }
>>>>>>> parent of ce2b3d1 (Merge pull request #25 from zombieking1555/one-controller-display)
}
