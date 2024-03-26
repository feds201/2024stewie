package frc.robot.subsystems.Vision.camera;


import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Vision.VisionABC;
import frc.robot.subsystems.vision_sys.utils.VisionObject;

public class Front_Camera extends VisionABC {
        @Override
        public void seedNetworkTables() {

        }

        @Override
        public void writePeriodicOutputs() {

        }

        @Override
        public void setupShuffleboard() {

        }

        private Front_Camera() { }
        private final static Front_Camera INSTANCE = new Front_Camera();

        @SuppressWarnings("WeakerAccess")
        public static Front_Camera getInstance() {
                return INSTANCE;
        }

        @Override
        public void simulationPeriodic() {
        }

        @Override
        public void periodic() {}

        @Override
        public boolean CheckTarget() {
                return false;
        }

        @Override
        public Translation2d GetTarget(VisionObject object) {
                return null;
        }

        @Override
        public void setPipeline(int pipeline) {

        }

        @Override
        public void setLEDMode(int mode) {

        }

        @Override
        public void setCamMode(int mode) {

        }

        @Override
        public void BlinkLED() {

        }
}

