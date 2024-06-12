package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Wrist;

public class RotateWristBasic extends Command {
    private final Wrist c_intake;
    private final DoubleSupplier c_voltage;
   
    public RotateWristBasic(Wrist intake, DoubleSupplier voltage) {
        c_intake = intake;
        c_voltage = voltage;

        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        c_intake.setWristVoltage(c_voltage.getAsDouble());
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        c_intake.setWristVoltage(0);
    }   
}
