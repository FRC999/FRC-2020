package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * TODO: write a method to check the encoder and figure out whch way to go to
 * get to zero in the shortest time
 */
public class ShooterSubsystem extends Subsystem {

  static WPI_TalonSRX shooterMotorController = new WPI_TalonSRX(RobotMap.shooterWheelMotorControllerID);
  static WPI_TalonSRX panMotorController = new WPI_TalonSRX(RobotMap.shooterPanMotorControllerID);
  static WPI_TalonSRX tiltMotorController = new WPI_TalonSRX(RobotMap.ShooterTiltMotorControllerID);
  //public SensorCollection turretEncoder;

  // double shooterSpeed = 0.5;

  NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

  /**
   * This is to compensate for the fact that the zero point of the turret is
   * unknown. This adjusts the zero point to point to the back of the robot.
   */
  int adjustedTurretPosition;

  public void configureShooterControllers() {
    // this.turretEncoder = turretEncoder;
    shooterMotorController.configFactoryDefault();
    panMotorController.configFactoryDefault();
    panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    panMotorController.configFeedbackNotContinuous(true, RobotMap.configureTimeoutMs);
    //panMotorController.turretEncoder.getPulseWidthRiseToFallUs()
    tiltMotorController.configFactoryDefault();
    tiltMotorController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    tiltMotorController.setSelectedSensorPosition(0);
     /*Johnson motors have a quadrature encoder with 178 ticks per revolution. Notes on wiring them to talons:
      type   |motor | breakout board
             --------------------
          5v |brown | red
  output 1/A |yellow| green
  output 2/B |green | yellow
      ground |blue  | black
    
    */
  }

  /**
   * Update the 'adjusted' turret position: the position we use to compenstate for
   * the fact that the zero point of the real turret may be in an awkward
   * location.
   */
  public void updateAdjustedTurretPosition() {
    adjustedTurretPosition = adjustTurretPosition(0);// TODO: get a method call to get the real pos.

  }

  /**
   * Convert from an absolute position (given by the encoder) to an adjusted
   * position (what we use)
   * 
   * @param absolute the encoder ticks in an absolute scale
   * @return the encoder ticks in the adjusted scale
   */
  public int adjustTurretPosition(int absolute) {
    int relative = absolute - RobotMap.shooterPanMotorEncoderOffset;
    if (relative < 0) {
      relative += RobotMap.shooterPanMotorEncoderTicksPerTurretRotation + 1;
    }
    return relative;
  }

  /**
   * Convert from the relative/adjusted position (what we use) to the absolute
   * position that the motor takes
   * 
   * @param relative the encoder ticks in a relative scale
   * @return encoder ticks in an absolute scale
   */
  public int deAdjustTurretPosition(int relative) {
    int absolute = relative + RobotMap.shooterPanMotorEncoderOffset;
    if (absolute > RobotMap.shooterPanMotorEncoderTicksPerTurretRotation) {
      absolute -= RobotMap.shooterPanMotorEncoderTicksPerTurretRotation + 1;
    }
    return absolute;
  }

  public int getPanEncoder() {
    return panMotorController.getSelectedSensorPosition();
  }

 

  //public int gettiltEncoder() {
   // return tiltMotorController.getSelectedSensorPosition();
  //}
  /** stops the shooter motor. */
  public void standby() {
    shooterMotorController.set(ControlMode.PercentOutput, 0);
  }

  public void shoot(double shooterSpeed) {
    shooterMotorController.set(ControlMode.PercentOutput, shooterSpeed);
  }

  public void pan(double pan) {
    panMotorController.set(ControlMode.PercentOutput, pan);
  }

  /**
   * gets the x-value of the center of the object the camera is looking at. 640 is
   * the maximum; if it returns 1000, the pi is not posting to NetworkTables.
   */
  public double getX() {
    return networkTableInstance.getTable("TestTable/PI").getEntry("X").getDouble(1000);// 640 is the maximum;
  }

  /**
   * gets the y-value of the center of the object the camera is looking at. 480 is
   * the maximum; if it returns 1000, the pi is not posting to NetworkTables.
   */
  public double getY() {
    return networkTableInstance.getTable("TestTable/PI").getEntry("Y").getDouble(1000);// 480 is the maximum; 
  }

  /**tests whether the current x-value of the object the robot sees is in the center of its field of view. Outputs the difference (current value - 320). */
  public double differenceFromMiddleX()
  {
return (getX() -( RobotMap.shooterXResolution/2));
  }
 /**tests whether the current y-value of the object the robot sees is in the center of its field of view. Outputs the difference (current value - 240). */
 public double differenceFromMiddleY()
 {
return (getY() - ( RobotMap.shooterXResolution/2));
 }

 public boolean inBounds (double num) {
    boolean state = false; 
  if (num != 1000) {
    state = true;
   }
   return state;
 }
 public boolean getCenteredX() {
   boolean retVal = false;
   if(Math.abs(differenceFromMiddleX()) <= RobotMap.shooterResolutionAcceptableError)
   {retVal = true;}
   return retVal;
 }

 public String whichSide(int num) {
   String state = "";
   if(inBounds(num)) {
    if(getCenteredX()) {
      state = "Center";
    } else if (differenceFromMiddleX() < 0){
      state = "Left";
    } else {
      state = "Right";
    }
   } else {
     state = "Out Of Bounds";
   }
   return state;
 }

 public void centerShooterOnTarget(int num) {
  switch (whichSide(num)) {
    case "Left" : {
      panMotorController.set(RobotMap.shooterPanSpeed);
      System.out.println("TARGET LEFT OF CENTER");
    }
    break;
    case "Center" : {
      panMotorController.set(0);
      System.out.println("TARGET IN CENTER");
    }
    break;
    case "Right" : {
      panMotorController.set((RobotMap.shooterPanSpeed)*-1);
      System.out.println("TARGET RIGHT OF CENTER");
    }
    break;
    case "Out Of Bounds" : {
      panMotorController.set(0);
      System.out.println("TARGET OUT OF BOUNDS");
    }
    default : {
      panMotorController.set(0);
      System.out.println("DEFAULT");
    }
  }
 }

 
/**"zero" in this case is the front of the robot. If it is shorter to turn the motor forwards, return 1. If it is shorter to turn reverse, return -1. */
public int getWhichWayToTurnToGetToZero() {
  int retVal = 0;
  if (adjustTurretPosition(getPanEncoder()) <= RobotMap.shooterPanMotorEncoderTicksPerTurretRotation / 2) {
    retVal = -1;
  } else {
    retVal = 1;
  }
  return retVal;
}
/**uses the corrected encoder scale */
public int getWhichWayToTurnToGetToAdjustedEncoderValue(double encoderTicksRequested) {
  int retVal = 0;
  double target = encoderTicksRequested;
  if (encoderTicksRequested > RobotMap.shooterPanMotorEncoderTicksBeforeRollover) {
    target = encoderTicksRequested - RobotMap.shooterPanMotorEncoderTicksBeforeRollover;
  } else if (encoderTicksRequested < 0) {
    target = RobotMap.shooterPanMotorEncoderTicksBeforeRollover + encoderTicksRequested;
  }

  int adjTurrPos =adjustTurretPosition(getPanEncoder());
  if ((adjTurrPos <= 20 + RobotMap.shooterPanMotorEncoderTicksPerTurretRotation / 2)
      && (adjTurrPos >= RobotMap.shooterPanMotorEncoderTicksPerTurretRotation / 2 - 20)) {
    if (target > adjTurrPos) {
      retVal = 1;
    } else {
      retVal = -1;
    }
  } else if (target > adjTurrPos) {
    if (target > adjTurrPos + RobotMap.shooterPanMotorEncoderTicksPerTurretRotation / 2) {
      retVal = -1;
    } else {
      retVal = 1;
    }
  } else if (target < adjTurrPos) {
    if (target > adjTurrPos - RobotMap.shooterPanMotorEncoderTicksPerTurretRotation / 2) {
      retVal = -1;
    } else {
      retVal = 1;
    }
  }

  return retVal;
}

public double getHeadingDegreesFromPanEncoderAdjustValue() {
  double retVal = 0;
  retVal =adjustTurretPosition( getPanEncoder()) * (360. / RobotMap.shooterPanMotorEncoderTicksPerTurretRotation);
  return retVal;
}

public double getPanEncoderAdjustValueFromHeading(double heading) {
  double retVal = 0;
  retVal = heading * (RobotMap.shooterPanMotorEncoderTicksPerTurretRotation/360);
  return retVal;
}
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

}