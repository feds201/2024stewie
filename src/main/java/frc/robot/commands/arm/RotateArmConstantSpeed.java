package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.leds.Leds;

public class RotateArmConstantSpeed extends Command {   
    private DoubleSupplier c_speed;
    private Arm c_arm;
 
 public RotateArmConstantSpeed (Arm arm, DoubleSupplier speed) {
    c_speed = speed;
    c_arm = arm;

    addRequirements(c_arm);
 }

 @Override
 public void initialize() {
 }

 @Override
 public void execute() {
   double powerValue=c_speed.getAsDouble();
   c_arm.rotateOrHold(powerValue*ArmConstants.kArmSpeedScaler);

 }

 @Override
 public void end(boolean interrupted) {
 }

 @Override
 public boolean isFinished() {
   return false;
 }




}