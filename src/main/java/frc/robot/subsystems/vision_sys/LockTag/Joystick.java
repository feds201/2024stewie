package frc.robot.subsystems.vision_sys.LockTag;

import edu.wpi.first.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;

public class Joystick implements RotationSource {

    RobotContainer robotContainer;
    @Override
    public double getR() {
        return -MathUtil.applyDeadband(robotContainer.driverController.getRightX(), SwerveConstants.MaxAngularRate);
    }
}