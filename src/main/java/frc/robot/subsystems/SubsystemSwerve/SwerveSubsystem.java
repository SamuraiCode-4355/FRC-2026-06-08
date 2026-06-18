package frc.robot.subsystems.SubsystemSwerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MechanismConstants;
import frc.robot.LimelightHelpers;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Meter;




public class SwerveSubsystem extends SubsystemBase {

File directory = new File(Filesystem.getDeployDirectory(),"swerve");
SwerveDrive swerveDrive;
private static SwerveSubsystem instance;




  public SwerveSubsystem() {

    boolean blueAlliance = false;
    Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(0))
                                       : new Pose2d(new Translation2d(Meter.of(16),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(180));

    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed, 
      startingPose);
      
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    setupPathPlanner();

    
  }

  public void setupPathPlanner()
  {
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      AutoBuilder.configure(
          swerveDrive::getPose,
          swerveDrive::resetOdometry,
          swerveDrive::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(

          new PIDConstants(MechanismConstants.kPAuto, MechanismConstants.kIAuto, MechanismConstants.kDAuto),
              new PIDConstants(MechanismConstants.kPAutoAngle, MechanismConstants.kIAutoAngle, MechanismConstants.kDAutoAngle)
          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
                           );

    } catch (Exception e)
    {
      e.printStackTrace();
    }

    PathfindingCommand.warmupCommand().schedule();
  }

  public static SwerveSubsystem getInstance(){
    if (instance == null) {
      instance = new SwerveSubsystem();
    }
    return instance;
  }

  public void lock()
  {
    swerveDrive.lockPose();
  }

  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

    public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Module oriented [0]", swerveDrive.getModulePositions()[0].angle.getDegrees());
    SmartDashboard.putNumber("Module oriented [1]", swerveDrive.getModulePositions()[1].angle.getDegrees());
    SmartDashboard.putNumber("Module oriented [2]", swerveDrive.getModulePositions()[2].angle.getDegrees());
    SmartDashboard.putNumber("Module oriented [3]", swerveDrive.getModulePositions()[3].angle.getDegrees());

    DriverStation.getAlliance().ifPresent(
        alliance -> SmartDashboard.putString("Alliance", alliance.toString())
    );
  }

    public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
  
public void driveFieldOriented(ChassisSpeeds velocity) {
swerveDrive.driveFieldOriented(velocity);
  }

  //--------------------------------LIMELIGHT----------------------------------------------

 PIDController alignPID = new PIDController(0.10, 0, 0.0003,0003);

public void autoAlign(double xSpeed, double ySpeed) {

    if (!LimelightHelpers.getTV("")) return;

    double tx = LimelightHelpers.getTX("");

    double rotation = alignPID.calculate(tx, -17);

    swerveDrive.driveFieldOriented(
        new ChassisSpeeds(
            xSpeed,
            ySpeed,
            rotation
        )
    );
}

public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
  return run(() -> 
  {swerveDrive.driveFieldOriented(velocity.get());
  });
}

}