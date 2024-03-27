// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Vision.VisionVariables;
import frc.robot.subsystems.Vision.camera.Front_Camera;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;


/** Add your docs here. */
public class DriveToGamePiece extends Command {

        private Front_Camera ll;
        private CommandSwerveDrivetrain drivetrain;
        private PIDController thetaController = new PIDController(4.0, 0, 0.05);
        public DriveToGamePiece(CommandSwerveDrivetrain drivetrain, Front_Camera ll) {
                addRequirements(drivetrain);
                this.drivetrain = drivetrain;
                this.ll = ll;
        }
        private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
                                                                 .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.01).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.01)
                                                                 .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private double thetaOutput = 0;
        private final double xOutput = 0.2; // Speed to drive towards note will increase after testing
        private final double yOutput = 0;
        private double setpoint = 0; // How far the camera is offset from the center in degrees

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
                thetaController.reset();
                thetaController.setTolerance(0.05);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
                if (ll.CheckTarget()){
                        setpoint = Math.toRadians(-VisionVariables.BackCam.target.getX())+ drivetrain.getState().Pose.getRotation().getRadians();
                        SmartDashboard.putNumber("Game Piece setpoint", setpoint);
                        thetaController.setSetpoint(setpoint);
                        if (!thetaController.atSetpoint() ){
                                thetaOutput = thetaController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), setpoint);
                                SmartDashboard.putNumber("theta output", thetaOutput);
                        }
                }

                drivetrain.setControl(drive.withVelocityX(-xOutput * TunerConstants.kSpeedAt12VoltsMps).withVelocityY(yOutput).withRotationalRate(thetaOutput));
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
                drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
                return false;
        }
}