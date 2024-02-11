package frc.robot.subsystems.vision_sys.camera;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision_sys.utils.VisionObject;
import frc.robot.subsystems.vision_sys.vision_sys;

public class BackCamera extends vision_sys {
    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }

    @Override
    public boolean CheckTarget() {
        return false;
    }

    @Override
    public Translation2d GetTarget(VisionObject note) {
        return null;
    }

}