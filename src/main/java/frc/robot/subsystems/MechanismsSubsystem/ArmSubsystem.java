
package frc.robot.subsystems.MechanismsSubsystem;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class ArmSubsystem extends SubsystemBase {
    private SparkMax m_rightArm;
    private SparkMax m_leftArm;
    private SparkMaxConfig m_ArmConfig1;
    private SparkMaxConfig m_ArmConfig2;


    public static ArmSubsystem instance;


  public ArmSubsystem() {

    m_rightArm = new SparkMax(MechanismConstants.kArmID1, MotorType.kBrushless);
    m_leftArm = new SparkMax(MechanismConstants.kArmID2, MotorType.kBrushless);
    
    m_ArmConfig1 = new SparkMaxConfig();
    m_ArmConfig1.smartCurrentLimit(40).idleMode(IdleMode.kCoast);
    m_ArmConfig2 = new SparkMaxConfig();
    m_ArmConfig2.smartCurrentLimit(40).idleMode(IdleMode.kCoast);

    m_ArmConfig1.follow(m_leftArm, true);
    m_rightArm.configure(m_ArmConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftArm.configure(m_ArmConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public static ArmSubsystem getInstance(){

    if(instance == null){
      instance = new ArmSubsystem();
    }
    return instance;
  }

  public void armDown(){
      m_leftArm.set(-.20);
  }
  public void armUp(){
      m_leftArm.set(.35);
  }

  public void stopArm(){
    m_rightArm.set(0);
    m_leftArm.set(0);
  }
  public void resetEncoder(){
    m_leftArm.getEncoder().setPosition(0);
    m_rightArm.getEncoder().setPosition(0);
  }

  public double getValue(){
    return m_leftArm.getEncoder().getPosition();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Position Right:", m_rightArm.getEncoder().getPosition());
    SmartDashboard.putNumber("Arm Encoder Position Left:", m_leftArm.getEncoder().getPosition());

  }
}
