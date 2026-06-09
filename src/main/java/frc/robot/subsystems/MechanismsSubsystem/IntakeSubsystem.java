package frc.robot.subsystems.MechanismsSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax m_IntakeMotor;
  public boolean isOn = false;

  private static IntakeSubsystem instance;
  private SparkMaxConfig m_IntakeConfig;


  public IntakeSubsystem() {

    m_IntakeMotor = new SparkMax(MechanismConstants.kIntakeID, MotorType.kBrushless); 

    m_IntakeConfig = new SparkMaxConfig();
    m_IntakeConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast);
    m_IntakeMotor.configure(m_IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public static IntakeSubsystem getInstance(){

    if(instance == null){
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  public void Feed(){
    m_IntakeMotor.set(0.9);
  }

  public void Reverse(){
    m_IntakeMotor.set(-0.75);
  }

  public void StopFeed(){
    m_IntakeMotor.set(0);
  }
  
  @Override
  public void periodic() {
  }
}
