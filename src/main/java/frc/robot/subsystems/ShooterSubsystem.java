package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class ShooterSubsystem extends Subsystem {

  static WPI_TalonSRX shooterMotorController = new WPI_TalonSRX(RobotMap.shooterWheelMotorController);
  static WPI_TalonSRX panMotorController = new WPI_TalonSRX(RobotMap.shooterPanMotorController);
  static WPI_TalonSRX tiltMotorController = new WPI_TalonSRX(RobotMap.ShooterTiltMotorController);
  //public SensorCollection turretEncoder;

  // double shooterSpeed = 0.5;

  NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

  public void configureShooterControllers(){
    //this.turretEncoder = turretEncoder;
    shooterMotorController.configFactoryDefault();
    panMotorController.configFactoryDefault();
    tiltMotorController.configFactoryDefault();
    panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    panMotorController.configFeedbackNotContinuous(true, RobotMap.configureTimeoutMs);
    //panMotorController.turretEncoder.getPulseWidthRiseToFallUs()
    //tiltMotorController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public int getpanEncoder() {
    return panMotorController.getSelectedSensorPosition();
  }

  public void zeroShooterEncoders() {
     panMotorController.setSelectedSensorPosition(0);
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

  //public void tilt(double tilt) {
    //tiltMotorController.set(ControlMode.PercentOutput, tilt);
  //}
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

 public boolean getCenteredX() {
   boolean retVal = false;
   if(Math.abs(differenceFromMiddleX()) <= RobotMap.shooterResolutionAcceptableError)
   {retVal = true;}
   return retVal;
 }
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

}
