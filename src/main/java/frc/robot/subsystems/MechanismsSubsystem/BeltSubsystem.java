package frc.robot.subsystems.MechanismsSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class BeltSubsystem extends SubsystemBase {

  private static BeltSubsystem m_subIndex;
  private SparkMax m_belt;
  private SparkMax m_balls;
  private SparkMax m_topIndex;
//  private SparkMax m_flexIndex;

  private SparkMaxConfig m_beltConfig;
  private SparkMaxConfig m_ballsConfig;
//  private SparkMaxConfig m_topIndexConfig;
//  private SparkMaxConfig m_flexIndexConfig;

  public BeltSubsystem() {

    m_belt = new SparkMax(MechanismConstants.kBeltID, MotorType.kBrushless);
    m_balls = new SparkMax(MechanismConstants.kBallsID, MotorType.kBrushless);
  //  m_topIndex = new SparkMax(18, MotorType.kBrushless);
  //  m_flexIndex = new SparkMax(19, MotorType.kBrushless);

    m_beltConfig = new SparkMaxConfig();
    m_ballsConfig = new SparkMaxConfig();
  //  m_topIndexConfig = new SparkMaxConfig();
  //  m_flexIndexConfig = new SparkMaxConfig();
    

    m_beltConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast);
    m_ballsConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast);
  //  m_topIndexConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast);
  //  m_flexIndexConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast);

    m_belt.configure(m_beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_balls.configure(m_ballsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //  m_topIndex.configure(m_topIndexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
 }

  public static BeltSubsystem getInstance(){

    if(m_subIndex == null)
      m_subIndex = new BeltSubsystem();
    return m_subIndex;
  }

  public void transportFuel(double power){
    m_belt.set(power);
  }

    public void UpFuel(double power){
    m_balls.set(-power);
  //  m_topIndex.set(-power);
  //  m_flexIndex.set(power);
  }

      public void DownFuel(){
    m_balls.set(-0.9);
  }


  public void stop(){
    m_belt.set(0.0);
    m_balls.set(0);
  //  m_topIndex.set(0);
  //  m_flexIndex.set(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Motor Ampers", m_belt.getAppliedOutput());
    SmartDashboard.putNumber("Pico", m_belt.getBusVoltage());


  }
}
