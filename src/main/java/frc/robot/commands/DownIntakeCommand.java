
package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MechanismsSubsystem.ArmSubsystem;

public class DownIntakeCommand extends Command {
  // private final CommandXboxController controller;
  public DownIntakeCommand() {
    // this.controller = xbox;
    addRequirements(ArmSubsystem.getInstance());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (DriverStation.isTeleopEnabled() && ArmSubsystem.getInstance().getValue() > -3.3 ){
      ArmSubsystem.getInstance().armDown();
    }
    else if (DriverStation.isAutonomousEnabled()){
      ArmSubsystem.getInstance().armDown();
    }
    else {
      ArmSubsystem.getInstance().stopArm();
    }

  }

  @Override
  public void end(boolean interrupted) {
  
    ArmSubsystem.getInstance().stopArm();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
