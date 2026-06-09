package frc.robot.subsystems.MechanismsSubsystem;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.math.Conversions;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MechanismConstants;

public class ShooterSubsystem extends SubsystemBase {

  public SparkMax m_shooter1;
  public SparkMax m_shooter2;
  public SparkMax m_shooter3;
  public SparkMax m_shooter4;

  private double distanceToTag;
  private double sPCalculate;

  private SparkMaxConfig m_shooter1Config;
  private SparkMaxConfig m_shooter2Config;
  private SparkMaxConfig m_shooter3Config;
  private SparkMaxConfig m_shooter4Config;

  private static ShooterSubsystem instance;

  private PIDController m_shoot1PID;

  private double originalSP = 2800;
  private double currentSP = 2800;

  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(
          MechanismConstants.kffS,
          MechanismConstants.kffV,
          MechanismConstants.kffA);

  private boolean pidEnabled = false;
  private boolean useLimelightSetpoint = true;

  private double manualOutput = 0.0;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_angle = Rotations.mutable(0);
  private final MutAngularVelocity m_velocity = RPM.mutable(0);

  private double zLimelight;

  private SysIdRoutine sysIdRoutine;

  public ShooterSubsystem() {

    m_shooter1 = new SparkMax(MechanismConstants.kShootmotor1, MotorType.kBrushless);
    m_shooter2 = new SparkMax(MechanismConstants.kShootmotor2, MotorType.kBrushless);
    m_shooter3 = new SparkMax(MechanismConstants.kShootmotor3, MotorType.kBrushless);
    m_shooter4 = new SparkMax(MechanismConstants.kShootmotor4, MotorType.kBrushless);

    m_shooter1Config = new SparkMaxConfig();
    m_shooter2Config = new SparkMaxConfig();
    m_shooter3Config = new SparkMaxConfig();
    m_shooter4Config = new SparkMaxConfig();

    m_shooter1Config.smartCurrentLimit(40);
    m_shooter2Config.smartCurrentLimit(40);
    m_shooter3Config.smartCurrentLimit(40);
    m_shooter4Config.smartCurrentLimit(40);

    m_shoot1PID =
        new PIDController(
            MechanismConstants.kPShoot,
            MechanismConstants.kIShoot,
            MechanismConstants.kDShoot);

    m_shoot1PID.setTolerance(2.0);
    m_shoot1PID.setSetpoint(getTargetRps());

    m_shooter2Config.follow(m_shooter1, true);
    m_shooter3Config.follow(m_shooter1, false);
    m_shooter4Config.follow(m_shooter1, true);

    m_shooter1Config.idleMode(IdleMode.kCoast);
    m_shooter2Config.idleMode(IdleMode.kCoast);
    m_shooter3Config.idleMode(IdleMode.kCoast);
    m_shooter4Config.idleMode(IdleMode.kCoast);

    m_shooter1.configure(
        m_shooter1Config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_shooter2.configure(
        m_shooter2Config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_shooter3.configure(
        m_shooter3Config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_shooter4.configure(
        m_shooter4Config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                volts -> m_shooter1.setVoltage(volts),
                log -> {
                  log.motor("shooterlog")
                      .voltage(
                          m_appliedVoltage.mut_replace(
                              m_shooter1.getAppliedOutput() * RobotController.getBatteryVoltage(),
                              Volts))
                      .angularPosition(
                          m_angle.mut_replace(
                              m_shooter1.getEncoder().getPosition(),
                              Rotations))
                      .angularVelocity(
                          m_velocity.mut_replace(
                              m_shooter1.getEncoder().getVelocity(),
                              RPM));
                },
                this));
  }

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  private double rpmToRps(double rpm) {
    return rpm / 60.0;
  }

  private double getTargetRps() {
    return -rpmToRps(currentSP);
  }

  public double getVelocityM1() {
    return m_shooter1.getEncoder().getVelocity();
  }

  public double getTarget() {
    return currentSP;
  }

  public double getSetPoint() {
    return currentSP;
  }

  public boolean atSetPoint() {
    return m_shoot1PID.atSetpoint();
  }

  public void setFixedRPM(double rpm) {
    useLimelightSetpoint = false;
    currentSP = rpm;
    m_shoot1PID.setSetpoint(getTargetRps());
  }

  public void enableLimelightMode() {
    useLimelightSetpoint = true;
  }

  public void disableLimelightMode() {
    useLimelightSetpoint = false;
  }

  public void TrenchRPM() {
    setFixedRPM(2200);
  }

  public void HubRPM() {
    setFixedRPM(1650);
  }

  public void Stop() {
    pidEnabled = false;
    manualOutput = 0.0;

    m_shooter1.set(0);
    m_shooter2.set(0);
    m_shooter3.set(0);
    m_shooter4.set(0);
  }

  public void shootManual(double power) {
    pidEnabled = false;
    manualOutput = power;
  }

  public void setVoltage(double volts) {
    pidEnabled = false;
    m_shooter1.setVoltage(volts);
  }

  public void enablePID(boolean enable) {
    if (enable && !pidEnabled) {
      m_shoot1PID.reset();
      m_shoot1PID.setSetpoint(getTargetRps());
    }
    pidEnabled = enable;
  }

  @Override
  public void periodic() {

    if (LimelightHelpers.getBotPose_TargetSpace("")[2] != 0) {
      zLimelight = LimelightHelpers.getBotPose_TargetSpace("")[2];
    }

    double outputVolts;
    double velocityRps = m_shooter1.getEncoder().getVelocity() / 60.0;

    if (pidEnabled) {

      if (useLimelightSetpoint) {
        currentSP = Conversions.distanceToRPM(Math.abs(zLimelight));
      }

      double targetRps = getTargetRps();
      m_shoot1PID.setSetpoint(targetRps);

      double pidOutput = m_shoot1PID.calculate(velocityRps);
      double ffOutput = ff.calculate(targetRps);

      outputVolts = ffOutput + pidOutput;
    } else {
      outputVolts = manualOutput * RobotController.getBatteryVoltage();
    }

    if (outputVolts > 10.0) {
      outputVolts = 10.0;
    } else if (outputVolts < -10.0) {
      outputVolts = -10.0;
    }

    m_shooter1.setVoltage(outputVolts);

    SmartDashboard.putNumber("Output", outputVolts);
    SmartDashboard.putNumber("RPM 1", m_shooter1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Setpoint RPM", currentSP);
    SmartDashboard.putNumber("Shooter Setpoint RPS", getTargetRps());
    SmartDashboard.putNumber("Shooter Velocity RPS", velocityRps);
    SmartDashboard.putBoolean("Shooter PID Enabled", pidEnabled);
    SmartDashboard.putBoolean("Shooter Using Limelight", useLimelightSetpoint);
    SmartDashboard.putBoolean("AtSetpoint", atSetPoint());
    SmartDashboard.putNumber("z", zLimelight);
    SmartDashboard.putNumber("sPCalculate", sPCalculate);
    SmartDashboard.putNumber("DistanceToTag", distanceToTag);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}