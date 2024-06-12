// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  /** Creates a new Leds. */
  private final Spark leds;

  private double currentColor;
  private static DriverStation.Alliance currentAlliance = null;

  public Leds() {
    leds = new Spark(0); // We treat the Leds as a servo which is coming in from PWM port 5

    currentColor = LedColors.FOREST;
  }

  public void setLEDToAllianceColor() {
    Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();

    if(allianceColor.isPresent()) {
      SmartDashboard.putString("current alliance color", allianceColor.get().toString());
    }

    if (allianceColor.isEmpty()) {
      currentColor = Leds.LedColors.GRAY;  
    } else if(allianceColor.get().equals(DriverStation.Alliance.Red)) {
      currentColor = Leds.LedColors.RED;
      currentAlliance = DriverStation.Alliance.Red;
    } else if (allianceColor.get().equals(DriverStation.Alliance.Blue)) {
      currentColor = Leds.LedColors.BLUE;
      currentAlliance = DriverStation.Alliance.Blue;
    } else {
      currentColor = -0.59; // FIRE!!
                            // ALSO THIS SHOULD *NEVER* HAPPEN
    }
  }

  public static double getAllianceColor() {
    if(currentAlliance != null && currentAlliance.equals(DriverStation.Alliance.Red)) {
      return LedColors.RED;
    } else if (currentAlliance != null && currentAlliance.equals(DriverStation.Alliance.Blue)) {
      return LedColors.BLUE;
    } else {
      return LedColors.FOREST;
    }
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
    if(currentColor == LedColors.FOREST) {
      setLEDToAllianceColor();
    }

    runLeds(getLedColor());
    SmartDashboard.putBoolean("ARE WE RUNNING??", leds.isAlive());
    SmartDashboard.putString("Current LED Color", LedColors.ColorToString(getLedColor()));
  }

  public static class LedColors {
    public final static double HotPink = 0.57;
    public final static double DarkRed = 0.59;
    public final static double RED = 0.61;
    public final static double RedOrange = 0.63;
    public final static double ORANGE = 0.65;
    public final static double Gold = 0.67;
    public final static double YELLOW = 0.69;
    public final static double LawnGreen = 0.71;
    public final static double Lime = 0.73;
    public final static double DarkGreen = 0.75;
    public final static double GREEN = 0.77;
    public final static double BlueGreen = 0.79;
    public final static double Aqua = 0.81;
    public final static double SkyBlue = 0.83;
    public final static double DarkBlue = 0.85;
    public final static double BLUE = 0.87;
    public final static double BlueViolet = 0.89;
    public final static double VIOLET = 0.91;
    public final static double WHITE = 0.93;
    public final static double GRAY  = 0.95;
    public final static double DarkGray = 0.97;
    public final static double BLACK = 0.99;
    public final static double NEUTRAL = -0.45; // Strobe color rainbow
    public final static double FOREST = -0.47; // FOREST!!

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
      if (color == NEUTRAL) {
        return "Neutral";
      }
      if (color == GRAY) {
        return "Gray";
      }
      if (color == FOREST) {
        return "Confetti";
      }
      return "Unknown";
    }
  }
}
