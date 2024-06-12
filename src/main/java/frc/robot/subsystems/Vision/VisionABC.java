/*
 * *****************************************************************************
 *  * Copyright (c) 2024 FEDS 201. All rights reserved.
 *  *
 *  * This codebase is the property of FEDS 201 Robotics Team.
 *  * Unauthorized copying, reproduction, or distribution of this code, or any
 *  * portion thereof, is strictly prohibited.
 *  *
 *  * This code is provided "as is" and without any express or implied warranties,
 *  * including, without limitation, the implied warranties of merchantability
 *  * and fitness for a particular purpose.
 *  *
 *  * For inquiries or permissions regarding the use of this code, please contact
 *  * feds201@gmail.com
 *  ****************************************************************************
 *
 */

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemABC;
import frc.robot.subsystems.Vision.utils.VisionObject;

public abstract class VisionABC extends SubsystemBase {



  public VisionABC() {}

  public abstract void periodic();
  public abstract boolean CheckTarget();
  public abstract Translation2d GetTarget(VisionObject object);
  public abstract void setPipeline(int pipeline);
  public abstract void setLEDMode(int mode);
  public abstract void setCamMode(int mode);
  public abstract Command BlinkLED();
  public abstract Command TurnOffLED();

  /**
   * This method is called periodically by the {@link CommandScheduler}. Useful for updating
   * subsystem-specific state that needs to be maintained for simulations, such as for updating
   * {@link edu.wpi.first.wpilibj.simulation} classes and setting simulated sensor readings.
   */
  @Override
  public abstract void simulationPeriodic();

  public Boolean enabled;
}
