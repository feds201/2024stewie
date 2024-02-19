package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class RotateWristBasic extends Command {

    private final Intake c_intake;
    private final double c_voltage;
    public RotateWristBasic(Intake intake, double voltage) {
        c_intake = intake;
        c_voltage = voltage;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        c_intake.setWristVoltage(c_voltage);
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
