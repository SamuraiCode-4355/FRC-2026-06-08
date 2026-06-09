package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MechanismsSubsystem.ArmSubsystem;
import frc.robot.subsystems.MechanismsSubsystem.BeltSubsystem;
import frc.robot.subsystems.MechanismsSubsystem.ShooterSubsystem;

public class ShooterCommand extends Command {

    private double targetPosition;
    private boolean goingUp = true;

    private static final double upValue = 1.0;
    private static final double tolerance = 0.1;
    private static final double rpmTolerance = 75.0;

    private final boolean useLimelight;
    private final double manualRPM;

    public ShooterCommand() {
        this.useLimelight = true;
        this.manualRPM = 0.0;
        addRequirements(ShooterSubsystem.getInstance(), BeltSubsystem.getInstance());
    }

    public ShooterCommand(double manualRPM) {
        this.useLimelight = false;
        this.manualRPM = manualRPM;
        addRequirements(ShooterSubsystem.getInstance(), BeltSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ShooterSubsystem shooter = ShooterSubsystem.getInstance();

        if (useLimelight) {
            shooter.enableLimelightMode();
        } else {
            shooter.setFixedRPM(manualRPM);
        }

        shooter.enablePID(true);
        targetPosition = ArmSubsystem.getInstance().getValue() + upValue;
    }

    @Override
    public void execute() {
        ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        BeltSubsystem belt = BeltSubsystem.getInstance();

        double currentRPM = Math.abs(shooter.getVelocityM1());
        double targetRPM = shooter.getTarget();

        if (currentRPM >= (targetRPM - rpmTolerance)) {
            belt.transportFuel(.9);
            belt.UpFuel(-.9);
        } else {
            belt.stop();
        }

        

        // double currentPos = ArmSubsystem.getInstance().getValue();

        // if (goingUp) {
        //    ArmSubsystem.getInstance().armUp();
        // } else {
        //    ArmSubsystem.getInstance().armDown();
        // }

        // if (currentPos - targetPosition < tolerance) {
        //     goingUp = !goingUp;
        //
        //     if (goingUp) {
        //         targetPosition = currentPos + upValue;
        //     } else {
        //         targetPosition = currentPos - upValue;
        //     }
        // }
    }

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().enablePID(false);
        ShooterSubsystem.getInstance().Stop();
        BeltSubsystem.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}