package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;


public class DIdJuliaNotPressbutton extends Command {
   boolean DIdJuliaNotPressbutton;
   public DIdJuliaNotPressbutton(boolean DIdJuliaNotPressbutton) {
      // each subsystem used by the command must be passed into the
      // addRequirements() method (which takes a vararg of Subsystem)
      this.DIdJuliaNotPressbutton = DIdJuliaNotPressbutton;
      addRequirements();
   }

   @Override
   public void initialize() {

   }

   @Override
   public void execute() {

      ArmConstants.DidJuiliaNotPressButton = DIdJuliaNotPressbutton;
   }

   @Override
   public boolean isFinished() {
      // TODO: Make this return true when this Command no longer needs to run execute()
      return false;
   }

   @Override
   public void end(boolean interrupted) {

   }
}
