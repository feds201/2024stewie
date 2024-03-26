package frc.robot.subsystems.Vision.camera;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.Vision.VisionABC;
import frc.robot.subsystems.Vision.VisionVariables;
import frc.robot.subsystems.Vision.utils.LimelightHelpers;
import frc.robot.subsystems.Vision.utils.RectanglePoseArea;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision_sys.utils.ObjectType;
import frc.robot.subsystems.vision_sys.utils.VisionObject;

import java.util.Random;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

public class Back_Camera extends VisionABC {
		public static String nt_key;
		public static VisionObject note;
		public Random random = new Random();
		CommandSwerveDrivetrain drivetrain;
		private Boolean enable = true;
		private Boolean trust = false;
		private double confidence = 0;
		private double compareDistance;
		private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Pose");
		private final DoubleArrayPublisher limelightPub = table.getDoubleArrayTopic("llPose").publish();
		private final RectanglePoseArea fieldBoundary = new RectanglePoseArea(new Translation2d(0, 0), new Translation2d(16.541, 8.211));
		public Back_Camera(CommandSwerveDrivetrain drivetrain) {
				this.drivetrain = drivetrain;
				nt_key = CameraConstants.BackCam.BACK_CAMERA_NETWORK_TABLES_NAME;
				LimelightHelpers.setPipelineIndex(nt_key, 0);
		}

		@Override
		public void periodic() {
				if ((enable || DriverStation.isDisabled()) && !RobotBase.isSimulation()) {
						// How to test:
						// Odometry is good on a nice flat surface so when testing on flat assume odometry as ground truth
						// Log over the last 5? seconds what has been the avg distance between odometry and the LL pose


						// Things to consider testing / excluding:
						// When spining past some rate we get bad results
						// Anything past 15ft seems to have too much variance
						//

						confidence = 0; // If we don't update confidence then we don't send the measurement
						LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(nt_key);
						SmartDashboard.putNumber("NumTags", limelightMeasurement.tagCount);

						// No tag found so check no further or pose not within field boundary
						if(limelightMeasurement.tagCount >= 1 && fieldBoundary.isPoseWithinArea(limelightMeasurement.pose)) {
								// Excluding different measurements that are absolute showstoppers even with full trust
								if(limelightMeasurement.avgTagDist < Units.feetToMeters(15) && drivetrain.getState().speeds.omegaRadiansPerSecond < Math.PI) {
										// Reasons to blindly trust as much as odometry
										if (trust || DriverStation.isDisabled() ||
												    (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagDist < Units.feetToMeters(10))) {
												confidence = 0.2;
												trust = false;
										} else {
												// High trust level anything less than this we shouldn't bother with
												compareDistance = limelightMeasurement.pose.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation());
												if( compareDistance < 0.5 ||
														    (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagDist < Units.feetToMeters(20)) ||
														    (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist < Units.feetToMeters(10))) {
														double tagDistance = Units.metersToFeet(limelightMeasurement.avgTagDist);
														// Double the distance for solo tag
														if (limelightMeasurement.tagCount == 1) {
																tagDistance = tagDistance * 2;
														}
														// Add up to .2 confidence depending on how far away
														confidence = 0.7 + (tagDistance / 100);
												}
										}
								}
						}
						addPose(limelightMeasurement, confidence);
				}
		}

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

		/**
		 * This method is called periodically by the {@link CommandScheduler}. Useful for updating
		 * subsystem-specific state that needs to be maintained for simulations, such as for updating
		 * {@link edu.wpi.first.wpilibj.simulation} classes and setting simulated sensor readings.
		 */
		@Override
		public void simulationPeriodic() { }

		private void addPose(LimelightHelpers.PoseEstimate limelightMeasurement, double confide) {
				if(confide > 0) {
						// We are publishing this to view as a ghost to try and help determine when not to use the LL measurements
						publishToField(limelightMeasurement);
						SmartDashboard.putBoolean("PoseUpdate", true);
						SmartDashboard.putNumber("LLConfidence", confide);
						drivetrain.addVisionMeasurement(
								limelightMeasurement.pose,
								limelightMeasurement.timestampSeconds,
								VecBuilder.fill(confide, confide, 99));
				} else {
						SmartDashboard.putBoolean("PoseUpdate", false);
						// We are publishing this to view as a ghost to try and help determine when not to use the LL measurements
						publishToField(new LimelightHelpers.PoseEstimate(new Pose2d(), 0, 0, 0, 0, 0, 0));
				}
		}

		private void publishToField(LimelightHelpers.PoseEstimate limelightMeasurement) {
				// If you have a Field2D you can easily push it that way here.
				limelightPub.set(new double[] {
						limelightMeasurement.pose.getX(),
						limelightMeasurement.pose.getY(),
						limelightMeasurement.pose.getRotation().getDegrees()
				});
		}

		public void useLimelight(boolean enable) {
				this.enable = enable;
		}

		public void trustLL(boolean trust) {
				this.trust = trust;
		}



}