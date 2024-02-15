// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.DistanceSensor;

// import com.revrobotics.Rev2mDistanceSensor;
// import edu.wpi.first.wpilibj.smartdashboard.*;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class DistanceSensor extends SubsystemBase {
//   private Rev2mDistanceSensor sensorOnboard;

//   public DistanceSensor(Rev2mDistanceSensor.Port portType) {
//     sensorOnboard = new Rev2mDistanceSensor(portType);
//     sensorOnboard.setAutomaticMode(true);

//     setupDebug();
//   }

//   @Override
//   public void periodic() {
//     SmartDashboard.putBoolean("Is onboard one range valid??", sensorOnboard.isRangeValid());
//     SmartDashboard.putBoolean("Is onboard alive????", sensorOnboard.isEnabled());
//     if (sensorOnboard.isRangeValid()) {
//       SmartDashboard.putNumber("RangeOnboard", sensorOnboard.getRange());
//       SmartDashboard.putNumber("TimestampOnboard", sensorOnboard.getTimestamp());

//       SmartDashboard.putString("Distance Units Onboard", sensorOnboard.getDistanceUnits().toString());
//       SmartDashboard.putNumber("Some other range idk Onboard", sensorOnboard.GetRange());

//       SmartDashboard.putBoolean("In Range Onboard", sensorOnboard.getRange() < 4);
//     }
//     // }
//     // if (sensorMXP.isRangeValid()) {
//     // SmartDashboard.putNumber("Range MXP", sensorMXP.getRange());
//     // SmartDashboard.putNumber("Timestamp MXP", sensorMXP.getTimestamp());

//     // SmartDashboard.putString("Distance Units MXP",
//     // sensorMXP.getDistanceUnits().toString());
//     // SmartDashboard.putNumber("Some other range idk MXP", sensorMXP.GetRange());

//     // SmartDashboard.putBoolean("In Range MXP", sensorMXP.getRange() < 4);
//     // }
//     // if(distOnboard.isRangeValid()) {
//     // SmartDashboard.putNumber("Range Onboard", distOnboard.getRange());
//     // SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp());
//     // }

//     // if(distMXP.isRangeValid()) {
//     // SmartDashboard.putNumber("Range MXP", distMXP.getRange());
//     // SmartDashboard.putNumber("Timestamp MXP", distMXP.getTimestamp());
//     // }

//   }

//   private void setupDebug() {

//   }

//   // public class DistanceSensor extends SubsystemBase {
//   // private final I2C i2c1;

//   // public DistanceSensor() {
//   // i2c1 = new I2C(I2C.Port.kOnboard, 0x52);

//   // }

//   // @Override
//   // public void periodic() {
//   // SmartDashboard.putNumber("Test", 1);
//   // SmartDashboard.putNumber("Connected DEvice", i2c1.getDeviceAddress());
//   // SmartDashboard.putNumber("Distance", getDistance());
//   // }

//   // public double getDistance() {
//   // byte[] buffer = new byte[2]; // Adjust buffer size if needed
//   // i2c1.read(0x52, 2, buffer); // Assuming distance is in 2 bytes at register
//   // 0x02
//   // double distance = ByteBuffer.wrap(buffer).getShort() / 10.0; // Example
//   // conversion
//   // return distance;
//   // }

// }
