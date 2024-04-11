package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterWheels;

public class ShootNoteMotionMagicVelocity extends Command {

    private final ShooterWheels c_shooterWheels;
    private final DoubleSupplier c_velocityTop;
    private final DoubleSupplier c_velocityBottom;

    public ShootNoteMotionMagicVelocity(ShooterWheels wheels, DoubleSupplier velocityTop, DoubleSupplier velocityBottom) {
        c_shooterWheels = wheels;
        c_velocityTop = velocityTop;
        c_velocityBottom = velocityBottom;

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
        c_shooterWheels.setShootVelocityMotionMagic(
            c_velocityTop.getAsDouble(),
            c_velocityBottom.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    
    
}
