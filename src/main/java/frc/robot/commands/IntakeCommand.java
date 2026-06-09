package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MechanismsSubsystem.BeltSubsystem;
import frc.robot.subsystems.MechanismsSubsystem.IntakeSubsystem;

public class IntakeCommand extends Command {

  private boolean reverse;

  public IntakeCommand(boolean Reverse) {
    
    addRequirements(IntakeSubsystem.getInstance());//, ShooterSubsystem.getInstance(), BeltSubsystem.getInstance());
    this.reverse = Reverse;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (reverse == true){
      IntakeSubsystem.getInstance().Reverse();
    }
    else{
      IntakeSubsystem.getInstance().Feed();
      BeltSubsystem.getInstance().transportFuel(.4);
      BeltSubsystem.getInstance().UpFuel(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.getInstance().StopFeed();
    BeltSubsystem.getInstance().stop();
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
