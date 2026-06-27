
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MechanismsSubsystem.ArmSubsystem;

public class resetEncoderCommand extends Command {
  public resetEncoderCommand() {
    addRequirements(ArmSubsystem.getInstance());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ArmSubsystem.getInstance().resetEncoder();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
