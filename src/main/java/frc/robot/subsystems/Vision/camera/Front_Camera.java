package frc.robot.subsystems.Vision.camera;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.Vision.VisionABC;
import frc.robot.subsystems.Vision.VisionVariables;
import frc.robot.subsystems.Vision.utils.LimelightHelpers;
import frc.robot.subsystems.Vision.utils.ObjectType;
import frc.robot.subsystems.Vision.utils.VisionObject;

import java.util.Random;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

public class Front_Camera extends VisionABC {
        public static String nt_key;
        private static final ShuffleboardTab tab = Shuffleboard.getTab("FrontCamera");
        public static NetworkTable table;
        public static VisionObject note;
        public Random random = new Random();
        public Front_Camera() {

                nt_key = CameraConstants.FrontCam.FRONT_CAMERA_NETWORK_TABLES_NAME;
                table = NetworkTableInstance.getDefault().getTable(nt_key);
                note = new VisionObject(0, 0, 0, ObjectType.NOTE);
        }

        private final static Front_Camera INSTANCE = new Front_Camera();

        @SuppressWarnings("WeakerAccess")
        public static Front_Camera getInstance() {
                return INSTANCE;
        }

        @Override
        public void simulationPeriodic() {
                note.update(
                        random.nextDouble() * 100,
                        random.nextDouble() * 100,
                        random.nextDouble() * 360
                );
                Periodic();
        }
        @Override
        public void periodic() {
                note.update(
                        table.getEntry("tx").getNumber(0).doubleValue(),
                        table.getEntry("ty").getNumber(0).doubleValue(),
                        table.getEntry("ta").getNumber(0).doubleValue()
                );
                Periodic();
        }

        @Override
        public boolean CheckTarget() {
                return table.getEntry("tv").getNumber(0).intValue() == 1;
        }

        @Override
        public Translation2d GetTarget(VisionObject object) {
                return new Translation2d(note.getX(), note.getY());
        }

        @Override
        public void setPipeline(int pipeline) {
                LimelightHelpers.setPipelineIndex(nt_key,pipeline);
        }

        @Override
        public void setLEDMode(int mode) {
                switch ( mode ) {
                        case 0:
                                LimelightHelpers.setLEDMode_ForceOff(nt_key);
                                break;
                        case 1:
                                LimelightHelpers.setLEDMode_ForceOn(nt_key);
                                break;
                        case 2:
                                LimelightHelpers.setLEDMode_ForceBlink(nt_key);
                                break;
                        default:
                                break;
                }
        }

        /**
         * @param mode 0: Driver Camera Mode, 1: Processor Camera Mode
         */
        @Override
        public void setCamMode(int mode){
                switch ( mode ) {
                        case 0:
                                LimelightHelpers.setCameraMode_Driver(nt_key);
                                break;
                        case 1:
                                LimelightHelpers.setCameraMode_Processor(nt_key);
                                break;
                        default:
                                break;
                }
        }

        @Override
        public Command BlinkLED() {
                return runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink(nt_key))
                               .andThen(waitSeconds(2))
                               .andThen(() -> LimelightHelpers.setLEDMode_ForceOff(nt_key));
        }

        @Override
        public Command TurnOffLED() {
                return runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(nt_key));
        }

        private void Periodic(){
//                VisionVariables.FrontCam.distance = note.getDistance();
                VisionVariables.FrontCam.tv = table.getEntry("tv").getNumber(0).intValue();
                VisionVariables.FrontCam.CameraMode = table.getEntry("camMode").getNumber(0).intValue();
                VisionVariables.FrontCam.LEDMode = table.getEntry("ledMode").getNumber(0).intValue();

        }
}

