package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MechanismsSubsystem.BeltSubsystem;
import frc.robot.subsystems.MechanismsSubsystem.ShooterSubsystem;

public class DownFuelCommand extends Command {

  public DownFuelCommand() {

    addRequirements(BeltSubsystem.getInstance(), ShooterSubsystem.getInstance());
  }

  @Override
  public void initialize() {

    
  }


  @Override
  public void execute() {

       BeltSubsystem.getInstance().DownFuel();
   
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
