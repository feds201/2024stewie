package frc.robot.subsystems.vision_sys.LockTag;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.OIConstants;
import frc.robot.constants.SwerveConstants;

public class Joystick implements RotationSource {
    public  final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverController);

    @Override
    public double getR() {
        return (driverController.getRightX() * SwerveConstants.MaxAngularRate);
    }
}