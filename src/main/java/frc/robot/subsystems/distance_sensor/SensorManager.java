// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.distance_sensor;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class SensorManager extends SubsystemBase {
    private DistanceSensor sensorOnBoard;
    private DistanceSensor sensorMXP;
    
    public SensorManager() {

    }

    public Command getMXP() {
        return run(() -> {
            sensorOnBoard = null;
            sensorMXP = new DistanceSensor(Port.kMXP);
        });

    }

    public Command getOnboard() {
        return run(() -> {
            sensorMXP = null;
            sensorOnBoard = new DistanceSensor(Port.kOnboard);
        });

    }
    // Called when the command is initially scheduled.

}
