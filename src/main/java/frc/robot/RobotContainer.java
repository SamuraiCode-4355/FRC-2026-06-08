package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.MechanismsSubsystem.ArmSubsystem;
import frc.robot.subsystems.MechanismsSubsystem.BeltSubsystem;
import frc.robot.subsystems.MechanismsSubsystem.ShooterSubsystem;
import frc.robot.subsystems.SubsystemSwerve.SwerveSubsystem;
import frc.robot.commands.BeltCommand;
import frc.robot.commands.DownFuelCommand;
import frc.robot.commands.DownIntakeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.UpIntakeCommand;
import swervelib.SwerveInputStream;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

private final SwerveSubsystem drivebase = new SwerveSubsystem();
// private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final SendableChooser<Command> autoChooser;
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController mechaController = new CommandXboxController(OperatorConstants.kMechaControllerPort);
  private final CommandPS4Controller Ps4Drive = new CommandPS4Controller(2);


  public RobotContainer() {

    configureBindings();

    NamedCommands.registerCommand("Shoot", new ShooterCommand().withTimeout(5));
    NamedCommands.registerCommand("Shoot2200", new ShooterCommand(1850).withTimeout(5));
    NamedCommands.registerCommand("Belt", new BeltCommand());
    NamedCommands.registerCommand("Belt", new BeltCommand());
    NamedCommands.registerCommand("IntakeUp", new UpIntakeCommand());
    NamedCommands.registerCommand("IntakeDown", new DownIntakeCommand().withTimeout(0.3));

    NamedCommands.registerCommand("Intake", new IntakeCommand(false).withTimeout(5));
    // NamedCommands.registerCommand("EnableVision", new SwerveSubsystem().enableVisionAlign());
    // NamedCommands.registerCommand("DisableVision", new SwerveSubsystem().disableVisionAlign());

     autoChooser = AutoBuilder.buildAutoChooser();

     SmartDashboard.putData("Auto Chooser", autoChooser);
  }

    //---------------------------------SwerveController------------------------------

  SwerveInputStream driveAngularVelocity =
  SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX())
                        .withControllerRotationAxis(() -> -driverController.getRightX())
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(1)
                        .scaleRotation(1)
                        .allianceRelativeControl(false);

 SwerveInputStream driveDirectAngle =
      SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                () -> -driverController.getLeftY(),
                                                  () -> -driverController.getLeftX())
                                              .withControllerHeadingAxis(
                                                  driverController::getRightX,
                                                  driverController::getRightY)
                                                .deadband(OperatorConstants.DEADBAND)
                                                .scaleTranslation(1)
                                                .scaleRotation(1)
                                                .headingWhile(true)
                                                .allianceRelativeControl(false);

Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);


private void configureBindings() {

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
//     shooter.setDefaultCommand(
//     new ShooterCommandNOPID(shooter)
// );

    //---------------------------------MechanismController------------------------------

    new Trigger(() -> mechaController.getRightTriggerAxis() > 0.1).whileTrue(new ShooterCommand());

    mechaController.x().whileTrue(new BeltCommand());
    mechaController.rightBumper().whileTrue(new DownIntakeCommand());
    mechaController.leftBumper().onTrue(new UpIntakeCommand());

    mechaController.b().whileTrue(new DownFuelCommand());

     new Trigger(() -> mechaController.getLeftTriggerAxis() > 0.1).whileTrue(new ShooterCommand(1700));    // Manual PID 1650
      mechaController.a().whileTrue(new ShooterCommand(3200));
        mechaController.y().whileTrue(new ShooterCommand(400));

    // mechaController.a().whileTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().toggleSetpoint()));
    // mechaController.povLeft().onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().sumeFive()));
    // mechaController.povRight().onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().restFive()));
    // mechaController.povDown().onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().restTeen()));
    // mechaController.povUp().onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().sumeTeen()));
    


    //----------------------------------XboxController-----------------------------------

    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1).toggleOnTrue(new IntakeCommand(false));
    driverController.povDown().whileTrue(new IntakeCommand(true));
    driverController.rightBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());    
    driverController.x().onTrue(new InstantCommand (drivebase::zeroGyro, drivebase));
    driverController.leftBumper().toggleOnTrue(
    drivebase.run(() ->
        drivebase.autoAlign(
            -driverController.getLeftY()*1.5,
            -driverController.getLeftX()*1.5
        )
    )
);

    //----------------------------------PS4Controller-----------------------------------

new Trigger(() -> Ps4Drive.getL2Axis() > 0.1).toggleOnTrue(new IntakeCommand(false));
    Ps4Drive.povDown().whileTrue(new IntakeCommand(true));
    Ps4Drive.R1().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());    
    Ps4Drive.square().onTrue(new InstantCommand (drivebase::zeroGyro, drivebase));
    Ps4Drive.L1().toggleOnTrue(
    drivebase.run(() ->
        drivebase.autoAlign(
            -Ps4Drive.getLeftY()*1.5,
            -Ps4Drive.getLeftX()*1.5
        )
    )
);
    // sysIdController.y().whileTrue(ShooterSubsystem.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // sysIdController.a().whileTrue(ShooterSubsystem.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // sysIdController.b().whileTrue(ShooterSubsystem.getInstance().sysIdDynamic(SysIdRoutine.Direction.kForward));
    // sysIdController.x().whileTrue(ShooterSubsystem.getInstance().sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }

}