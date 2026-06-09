
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MechanismsSubsystem.ArmSubsystem;

public class UpIntakeCommand extends Command {
  // private final CommandXboxController controller;
  public UpIntakeCommand() {
    // this.controller = xbox;
    addRequirements(ArmSubsystem.getInstance());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (ArmSubsystem.getInstance().getValue() < -0.1){
      ArmSubsystem.getInstance().armUp();
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
