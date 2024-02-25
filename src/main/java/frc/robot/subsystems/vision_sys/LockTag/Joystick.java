package frc.robot.subsystems.vision_sys.LockTag;

import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;

public class Joystick implements RotationSource {

    RobotContainer robotContainer;
    @Override
    public double getR() {
        return (-robotContainer.driverController.getRightX() * SwerveConstants.MaxAngularRate);
    }
}