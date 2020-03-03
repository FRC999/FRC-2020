/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
/**
 * Add your docs here. TODO: Add docs
 */
public class TalonDriveSubsystem extends DriveSubsystemBase {
  // Put methods for controlling this subsystem here. Call these from Commands.

  //For isOnTarget
  boolean wasOnTarget = false;
  int withinAcceptableErrorLoops = 0;

  static WPI_TalonSRX frontLeftDriveTalonSRX = new WPI_TalonSRX(RobotMap.frontLeftDriveMotorControllerID);
  static WPI_TalonSRX backLeftDriveTalonSRX = new WPI_TalonSRX(RobotMap.backLeftDriveMotorControllerID);
  static WPI_TalonSRX frontRightDriveTalonSRX = new WPI_TalonSRX(RobotMap.frontRightDriveMotorControllerID);
  static WPI_TalonSRX backRightDriveTalonSRX = new WPI_TalonSRX(RobotMap.backRightDriveMotorControllerID);

  public TalonDriveSubsystem(){
	  frontLeftDriveMotorController = frontLeftDriveTalonSRX;
	  backLeftDriveMotorController = backLeftDriveTalonSRX;
	  frontRightDriveMotorController = frontRightDriveTalonSRX;
    backRightDriveMotorController = backRightDriveTalonSRX;
    drive = new DifferentialDrive(frontLeftDriveTalonSRX, frontRightDriveTalonSRX);
  }

  /**
   * Sets the talons to our preferred defaults
   * We are going away from controller-groups, and back to master-slave
   * Call this in robot-init: it preforms basic setup for ArcadeDrive
   */
  public void resetDriveTrainControllers() {
    super.resetDriveTrainControllers();
  }

  public void configureEncoders(){
    frontLeftDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    frontRightDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  public void configureDriveTrainControllersForSimpleMagic(){
    super.configureDriveTrainControllersForSimpleMagic();
  } // End configureDriveTrainControllersForSimpleMagic
}
