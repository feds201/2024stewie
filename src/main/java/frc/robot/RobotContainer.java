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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.intake.RotateWristToPosition;
import frc.robot.commands.intake.RotateWristToPositionInfinite;
import frc.robot.commands.intake.RunIntakeWheels;
import frc.robot.commands.arm.RotateArmToPosition;
import frc.robot.commands.arm.RotateArmConstantSpeed;
import frc.robot.commands.arm.RotateArmManual;
import frc.robot.commands.autons.DriveForwardForTime;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.compound.*;
import frc.robot.commands.controller.ToggleRumble;
import frc.robot.commands.leds.SetLEDColor;
import frc.robot.commands.shooter.*;
import frc.robot.commands.swerve.AimToAprilTag;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Vision.camera.Back_Camera;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.intake.IntakeIRSensor;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.shooter.ShooterServos;
import frc.robot.subsystems.shooter.ShooterIRSensor;
import frc.robot.subsystems.shooter.ShooterRotation;
import frc.robot.subsystems.shooter.ShooterWheels;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
//import frc.robot.subsystems.vision_sys.VisionVariables.ExportedVariables;
//import frc.robot.subsystems.vision_sys.camera.BackCamera;
//import frc.robot.subsystems.vision_sys.utils.DashBoardManager;
import frc.robot.subsystems.Vision.VisionVariables;
import frc.robot.utils.LimelightUtils;
import frc.robot.utils.Telemetry;

import static frc.robot.constants.IntakeConstants.kIntakeNoteWheelSpeed;

public class RobotContainer {
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(SwerveConstants.MaxSpeed * 0.1)
        .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle autoAim = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);
    
    public double swerveSpeedMultiplier = 1;
    
    // SHOOTER
    private final ShooterWheels shooterWheels;
    private final ShooterRotation shooterRotation;
    private final ShooterServos servos;
    private final ShooterIRSensor shooterIRSensor;
    
    // INTAKE
    private final Wrist wrist;
    private final IntakeWheels intakeWheels;
    private final IntakeIRSensor intakeIRSensor;
    
    // ARM
    private final Arm arm;
    
    // Climber
    private final Climber climber;
    public final Leds leds;
    
    private final Back_Camera shooterSideCamera;
    
    public final CommandXboxController driverController;
    public final CommandXboxController operatorController;
    
    SendableChooser<Command> autonChooser = new SendableChooser<>();
    
    ShuffleboardTab commandsTab = Shuffleboard.getTab("commands");
    
    public RobotContainer() {
        // ARM
        arm = new Arm();
        
        // SHOOTER
        shooterWheels = new ShooterWheels();
        shooterRotation = new ShooterRotation(arm::getArmAngle);
        servos = new ShooterServos();
        shooterIRSensor = new ShooterIRSensor();
        
        // CLIMBER
        climber = new Climber();
        
        // INTAKE
        wrist = new Wrist();
        intakeWheels = new IntakeWheels();
        intakeIRSensor = new IntakeIRSensor();
        
        // LEDS
        leds = new Leds();
        
        // CAMERAS
        shooterSideCamera = new Back_Camera();
        
        // Add subsystems to their respective shuffleboard tabs
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
        
        setupArmCommands();
        setupClimberCommands();
        setupIntakeCommands();
        setupShooterCommands();
        setupErrorTriggers();
        setupAutonCommands();
    }
    
    private void registerAllAutoCommands() {
        NamedCommands.registerCommand("ShootNoteAtSpeakerOnly",
            new ShootNoteAtSpeakerOnly(shooterRotation, shooterWheels, servos, leds, () -> VisionVariables.ExportedVariables.Distance, shooterIRSensor));
        NamedCommands.registerCommand("DeployIntake",
            new DeployIntake(wrist, intakeWheels, shooterRotation, intakeIRSensor, leds, driverController, operatorController));
        NamedCommands.registerCommand("AlignShooterAndIntake",
            new AlignShooterAndIntake(shooterRotation, wrist, intakeWheels, servos,
                shooterIRSensor, leds));
        NamedCommands.registerCommand("ShootFromHandoff", //
            new ShootFromHandoff(shooterRotation, shooterWheels, servos, leds, () -> VisionVariables.ExportedVariables.Distance, shooterIRSensor).until(shooterIRSensor::getBeamBroken));
        NamedCommands.registerCommand("StopShooterWheelsPls",
            new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0));
        NamedCommands.registerCommand("AimToAprilTag", new AimToAprilTag(drivetrain, driverController::getLeftX,
            driverController::getLeftY, () -> VisionVariables.ExportedVariables.Distance));
        NamedCommands.registerCommand("ArmDown",
            new RotateArmToPosition(arm, () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint));
        // NamedCommands.registerCommand("FeedNoteToShooter", new
        // AlignShooterAndIntake(shooterRotation, wrist, intakeWheels,
        // servos, breakBeamSensorShooter, leds));
        NamedCommands.registerCommand("DropIntake", new DropIntake(wrist, arm));
        NamedCommands.registerCommand("SetFieldRelative", new InstantCommand(drivetrain::seedFieldRelative));
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
                        shooterWheels, servos, leds, () -> VisionVariables.ExportedVariables.Distance, shooterIRSensor))).until(shooterIRSensor::getBeamBroken)
                .andThen(
                    new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0)
                ));
        
        autonChooser.addOption("Step Back Fade Away",
            new ParallelCommandGroup(
                new DriveForwardForTime(drivetrain, 2),
                new RotateArmToPosition(arm,
                    () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint),
                new SequentialCommandGroup(
                    new WaitCommand(6),
                    new ShootNoteAtSpeakerOnly(shooterRotation,
                        shooterWheels, servos, leds, () -> VisionVariables.ExportedVariables.Distance, shooterIRSensor).until(shooterIRSensor::getBeamBroken),
                    new WaitCommand(2),
                    new ParallelCommandGroup(
                        new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0)))));
        
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
                            shooterWheels, servos, leds, () -> VisionVariables.ExportedVariables.Distance, shooterIRSensor))),
                new DriveForwardForTime(drivetrain, 5)));
        
        autonChooser.addOption("Shoot and Scram",
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(2),
                    new RotateArmToPosition(arm,
                        () -> ArmConstants.ArmPIDForExternalEncoder.kArmRotationFeederSetpoint),
                    new SequentialCommandGroup(
                        new WaitCommand(0.7),
                        new ShootNoteAtSpeakerOnly(
                            shooterRotation,
                            shooterWheels, servos, leds, () -> VisionVariables.ExportedVariables.Distance, shooterIRSensor))),
                new ParallelCommandGroup(
                    new RotateShooterBasic(shooterRotation, () -> 0),
                    new DriveForwardForTime(drivetrain, 6))));
        
        autonChooser.addOption("Red-2Note-CenterStart",
            drivetrain.getAutoPath("Red-2Note-CenterStart"));
        
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
        
        Shuffleboard.getTab("autons").add(autonChooser);
    }
    
    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new ParallelCommandGroup(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-driverController.getLeftY()
                    * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                .withVelocityY(-driverController.getLeftX()
                    * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                .withRotationalRate(-driverController.getRightX() *
                    SwerveConstants.MaxAngularRate * swerveSpeedMultiplier)),
            new RepeatCommand(
                new InstantCommand(this::printCurrentStickValues))));
        
        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
                Rotation2d.fromDegrees(90)));
        }
        
        drivetrain
            .registerTelemetry(
                logger::telemeterize);
        
        arm.setDefaultCommand(
            new RotateArmManual(
                arm,
                () -> 0));
        
        new Trigger(() -> arm.getArmAngle() > ArmConstants.kArmClimbLimit)
            .onTrue(new InstantCommand(() -> leds.setLedColor(Leds.LedColors.YELLOW)))
            .onFalse(new InstantCommand(() -> leds.setLedColor(Leds.getAllianceColor())));
        
        shooterWheels.setDefaultCommand(new ShootNoteMotionMagicVelocity(shooterWheels, () -> 0, () -> 0));
    }
    
    private void configureDriverController() {
        // reset the field-centric heading on left bumper press
         // LOAD BUTTON
         driverController.x()
         .onTrue(new ParallelCommandGroup(
             new RotateArmToPosition(arm, () -> 0),
             new AlignShooterAndIntake(shooterRotation, wrist, intakeWheels,
                 servos, shooterIRSensor, leds)));
     
     driverController.b()
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
     
     
     driverController.y()
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
     
     driverController.a()
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
     driverController.povLeft()
         .onTrue(new RotateWristToPosition(wrist, IntakeConstants.WristPID.kWristShooterFeederSetpoint));
     // new Trigger(breakBeamSensorShooter::getBeamBroken).onTrue(new
     // SetLEDColor(leds, Leds.LedColors.ORANGE));
//        operatorController.povUp()
//            .onTrue()

        driverController.povUp()
            .whileTrue(new RotateArmConstantSpeed(arm, ()-> 0.66));
        driverController.povDown()
            .whileTrue(new RotateArmConstantSpeed(arm, ()-> -0.66));
            
        





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
        
        driverController.rightBumper()
            .onTrue(new SpitOutNote(wrist, intakeWheels))
            .onFalse(new ResetIntake(wrist, intakeWheels));
        
        driverController.leftBumper()
            .onTrue(new RunIntakeWheels(intakeWheels, () -> kIntakeNoteWheelSpeed))
            .onFalse(new RunIntakeWheels(intakeWheels, () -> 0));
        
        driverController.povRight()
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
        
        // driverController.leftBumper()
        //     .onTrue(new InstantCommand(() -> swerveSpeedMultiplier = 0.2))
        //     .onFalse(new InstantCommand(() -> swerveSpeedMultiplier = 1));
    }
    
    public void configureOperatorController() {
       
    
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
}
