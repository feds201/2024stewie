// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class RotateArmToPosition extends Command {
    /** Creates a new RotateArm. */
    private final Arm c_arm;
    private final DoubleSupplier c_angle;
    private final boolean c_failure;

    public RotateArmToPosition(Arm arm, DoubleSupplier desiredAngle) {
        c_arm = arm;
        c_angle = desiredAngle;
        addRequirements(c_arm);

        if(desiredAngle.getAsDouble() < -20) {
            c_failure = true;
        } else {
            c_failure = false;
        }

        SmartDashboard.putBoolean("ARM FAILURE", c_failure);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        c_arm.setPIDTarget(c_angle.getAsDouble());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!c_failure) {
            c_arm.rotateArmToTarget();
        } else {
            c_arm.setFailure(true);
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        c_arm.rotateOrHold(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return c_failure || c_arm.isArmAtTarget();
    }
}
