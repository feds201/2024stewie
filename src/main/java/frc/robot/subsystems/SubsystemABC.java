package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robotSettings.NetworkTableConstants;

public abstract class SubsystemABC extends SubsystemBase {
   protected ShuffleboardTab tab;
   protected ShuffleboardLayout commands;
   protected NetworkTable ntTable;
   protected DataLog log;

   public abstract void seedNetworkTables();

   public abstract void writePeriodicOutputs();

   public abstract void setupTestCommands();

   public abstract void setupShuffleboard();

  

   public SubsystemABC(String subsystemName) {
      tab = Shuffleboard.getTab(subsystemName);
      ntTable = NetworkTableConstants.inst.getTable(subsystemName);
      log = DataLogManager.getLog();
      setupShuffleboard();
      setupTestCommands();
   }
}