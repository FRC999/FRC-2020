package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class ShooterSubsystem extends Subsystem {

  static WPI_TalonSRX shooterMotorController = new WPI_TalonSRX(RobotMap.shooterWheelMotorController);
  static WPI_TalonSRX panMotorController = new WPI_TalonSRX(RobotMap.shooterPanMotorController);
  static WPI_TalonSRX tiltMotorController = new WPI_TalonSRX(RobotMap.ShooterTiltMotorController);
  //public SensorCollection turretEncoder;

  // double shooterSpeed = 0.5;

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

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

}
