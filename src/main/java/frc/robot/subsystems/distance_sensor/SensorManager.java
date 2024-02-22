// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.distance_sensor;

import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class SensorManager extends SubsystemBase {
    private boolean isMXPActive;

    private DistanceSensor sensorOnboard = null;
    private DistanceSensor sensorMXP = null;
    
    public SensorManager(boolean isMXP) {
        isMXPActive = isMXP;

        if(isMXPActive) {
            //setMXP();
        } else {
            //setOnboard();
        }
    }

    public DistanceSensor getActive() {
        if(isMXPActive) {
            return sensorMXP;
        } else {
            return sensorOnboard;
        }
    }

    public void setMXP() {
        sensorOnboard = null;
        System.gc();
        sensorMXP = new DistanceSensor(Port.kMXP);
        isMXPActive = true;
    }

    public void setOnboard() {
        sensorMXP = null;
        System.gc();
        sensorOnboard = new DistanceSensor(Port.kOnboard);
        isMXPActive = false;
    }
}
