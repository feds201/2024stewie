// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  /** Creates a new Leds. */
  private final Spark leds;
  
  private double currentColor;

  public Leds() {
    leds = new Spark(3); // We treat the Leds as a servo which is coming in from PWM port 6
    currentColor = 0.59;
  }

  public void setLedColor(double color) {
    // ONLY BETWEEN -1 and 1
    this.currentColor = color;
  }

  public double getLedColor() {
    return this.currentColor;
  }

  private void runLeds(double color) {
    leds.set(color);
  }

  @Override
  public void periodic() {
    runLeds(getLedColor());
    SmartDashboard.putBoolean("ARE WE RUNNING??", leds.isAlive());
    SmartDashboard.putString("Current LED Color", LedColors.ColorToString(getLedColor()));
  }

  public static class LedColors {
    public final static double RED = 0.61;
    public final static double ORANGE = 0.65;
    public final static double YELLOW = 0.69;
    public final static double GREEN = 0.77;
    public final static double BLUE = 0.87;
    public final static double VIOLET = 0.91;
    public final static double WHITE = 0.93;
    public final static double BLACK = 0.99;

    public static String ColorToString(double color) {
      if (color == RED) {
        return "Red";
      }
      if (color == ORANGE) {
        return "Orange";
      }
      if (color == YELLOW) {
        return "Yellow";
      }
      if (color == GREEN) {
        return "Green";
      }
      if (color == BLUE) {
        return "Blue";
      }
      if (color == BLACK) {
        return "Black";
      }
      if (color == WHITE) {
        return "White";
      }
      if (color == VIOLET) {
        return "Violet";
      }
      if (color == RED) {
        return "Red";
      }
      return "Unknown";
    }
  }
}
