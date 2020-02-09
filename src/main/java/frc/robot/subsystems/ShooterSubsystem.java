package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends Subsystem {

  static WPI_TalonSRX shooterMotorController = new WPI_TalonSRX(RobotMap.shooterWheelMotorController);
  static WPI_TalonSRX panMotorController = new WPI_TalonSRX(RobotMap.shooterPanMotorController);
  static WPI_TalonSRX tiltMotorController = new WPI_TalonSRX(RobotMap.ShooterTiltMotorController);

  //double shooterSpeed = 0.5;

  public void configureShooterControllers(){
    shooterMotorController.configFactoryDefault();
    panMotorController.configFactoryDefault();
    tiltMotorController.configFactoryDefault();
    panMotorController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    tiltMotorController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public int getpanEncoder() {
    return panMotorController.getSelectedSensorPosition();
  }

  public int gettiltEncoder() {
    return tiltMotorController.getSelectedSensorPosition();
  }

  public void standby() {
    shooterMotorController.set(ControlMode.PercentOutput, 0);
  }

  public void shoot(double shooterSpeed) {
    shooterMotorController.set(ControlMode.PercentOutput, shooterSpeed);
  }

  public void pan(double pan) {
    panMotorController.set(ControlMode.PercentOutput, pan);
  }

  public void tilt(double tilt) {
    tiltMotorController.set(ControlMode.PercentOutput, tilt);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

}
