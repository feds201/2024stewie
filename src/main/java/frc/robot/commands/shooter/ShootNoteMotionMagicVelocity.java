package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterWheels;

public class ShootNoteMotionMagicVelocity extends Command {

    private final ShooterWheels c_shooterWheels;
    private final DoubleSupplier c_velocity;

    public ShootNoteMotionMagicVelocity(ShooterWheels wheels, DoubleSupplier velocity) {
        c_shooterWheels = wheels;
        c_velocity = velocity;

        addRequirements(c_shooterWheels);
    }

    @Override
    public void end(boolean interrupted) {
        // nothing should happen here -- to stop the motor, call this command with a value of 0
    }

    @Override
    public void execute() {
        // nothing should happen here -- no need to keep calling the same thing, it will go to completion
    }

    @Override
    public void initialize() {
        c_shooterWheels.setShootVelocityMotionMagic(c_velocity.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    
    
}
