// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class RotateArm extends Command {
    /** Creates a new RotateArm. */
    private final Arm c_arm;
    private final double c_angle;

    public RotateArm(Arm arm, double desiredAngle) {
        c_arm = arm;
        c_angle = desiredAngle;
        addRequirements(c_arm);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        c_arm.resetEncoder();
        c_arm.setPIDTarget(c_angle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        c_arm.rotateArmToTarget();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        c_arm.stopArmRotation();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; //c_arm.isArmAtTarget();
    }
}
