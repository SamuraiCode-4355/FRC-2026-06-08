package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MechanismsSubsystem.BeltSubsystem;
import frc.robot.subsystems.MechanismsSubsystem.ShooterSubsystem;

public class BeltCommand extends Command {

  public BeltCommand() {

    addRequirements(BeltSubsystem.getInstance(), ShooterSubsystem.getInstance());
    // addRequirements(ShooterSubsystem.getInstance());
  }

  @Override
  public void initialize() {

    
  }


  @Override
  public void execute() {

       BeltSubsystem.getInstance().transportFuel(.9);
       BeltSubsystem.getInstance().UpFuel(-.9);
   
   }

  @Override
  public void end(boolean interrupted) {

    BeltSubsystem.getInstance().stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
