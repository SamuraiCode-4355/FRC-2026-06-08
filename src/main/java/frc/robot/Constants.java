package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kMechaControllerPort = 1;
    public static final int kPS4DriverControllerPort = 2;
    public static final double DEADBAND = .05;
    
  }
  
  public static class MechanismConstants{

    public static final int kBeltID = 9;
    public static final int kIntakeID = 10;
    public static final int kArmID1 = 37;
    public static final int kArmID2 = 12;
    public static final int kBallsID = 13;
    public static final int kShootmotor1 = 14; //R_DOWN
    public static final int kShootmotor2 = 15; //L_DOWN  
    public static final int kShootmotor3 = 16; //R_UP
    public static final int kShootmotor4 = 17; //L_UP  
    public static final int kleftSupport = 30;
    public static final int krightSupport = 32;

    public static final double kPShoot = 0.0000003; //0.0003
    public static final double kIShoot = 0.0002; //0.002
    public static final double kDShoot = 0.000005; //0.00005

    public static final double kPAuto = 5.0; 
    public static final double kIAuto = 0.0; //0.00019
    public static final double kDAuto = 0.0;  

    public static final double kPAutoAngle = 5.0; 
    public static final double kIAutoAngle = 0.0; //0.00019
    public static final double kDAutoAngle = 0.0; 

    public static final double kIntakeUp = 0;
    public static final double kIntakeDown = 6.1;

    public static final double kffS = 0.15723;
    public static final double kffV = 0.12486;
    public static final double kffA = 0.025217;

  }


  public static class FieldConstants {
    public static final double aprilTagHubHeight = 1.12395 - 0.15;
    
  }


public static final double maxSpeed = Units.feetToMeters(8.5);

}