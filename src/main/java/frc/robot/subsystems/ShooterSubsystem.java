package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ShooterSubsystem extends Subsystem {

  public static WPI_TalonSRX shooterMotorController = new WPI_TalonSRX(RobotMap.shooterWheelMotorControllerID);
  public static WPI_TalonSRX panMotorController = new WPI_TalonSRX(RobotMap.shooterPanMotorControllerID);
  public static WPI_TalonSRX tiltMotorController = new WPI_TalonSRX(RobotMap.ShooterTiltMotorControllerID);
  //public SensorCollection turretEncoder;

  // double shooterSpeed = 0.5;

  NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

  public void configureShooterControllers(){
    //this.turretEncoder = turretEncoder;
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

  public int getpanEncoder() {
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


  


  /** gets the x-value of the center of the object the camera is looking at. 640 is the maximum; if it returns 1000, the pi is not posting to networktables.*/
  public double getX() {
   return networkTableInstance.getTable("TestTable/PI").getEntry("X").getDouble(1000);// 640 is the maximum; 
  }

  /** gets the y-value of the center of the object the camera is looking at.  480 is the maximum; if it returns 1000, the pi is not posting to networktables.*/
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

 /**the Pi returns 1000 when it doesn't see an object, so if the parameter is not 1000, the object is visible.
  * @param num the value to test for visibility from the pi.
  * @return true if visible.
 */

 public boolean inBounds (double num) {
    boolean state = false; 
  if (num != 1000) {
    state = true;
   }
   return state;
 }

/**
 * checks if the object the pi detects is within a predefined center range.
 * @return true if the object is in the acceptable center range.
 */

 public boolean getCenteredX() {
   boolean retVal = false;
   if(Math.abs(differenceFromMiddleX()) <= RobotMap.shooterResolutionAcceptableError)
   {retVal = true;}
   return retVal;
 }

/**
 * checks if the object detected by the pi is in bounds, and which side of center it is on (including in the center)
 * @param num the value to test
 * @return a string stating the object's location (either "Left", "Center", "Right", or "Out Of Bounds")
 */

 public String whichSide(double num) {
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
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

}