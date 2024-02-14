// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class RotateArm extends Command {
    /** Creates a new RotateArm. */
    private final Arm c_arm;

    private final double c_angle;
    private final PIDController pid;
    private final double TOLERANCE = 0.5;

    final double kP = 0.0;
    final double kI = 0.0;
    final double kD = 0.0;

    public RotateArm(Arm arm, double intakeAngle) {
        c_arm = arm;
        c_angle = intakeAngle;
        pid = new PIDController(kP, kI, kD);
        addRequirements(c_arm);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        c_arm.resetEncoder();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double output = pid.calculate(c_arm.getArmAngle(), c_angle);
        c_arm.rotateArm(output);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        c_arm.rotateArm(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(c_arm.getArmAngle() - c_angle) < TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }
}
