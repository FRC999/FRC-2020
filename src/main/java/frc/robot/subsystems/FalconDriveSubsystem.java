/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class FalconDriveSubsystem extends DriveSubsystemBase {
  //For isOnTarget
  boolean wasOnTarget = false;
  int withinAcceptableErrorLoops = 0;

  static WPI_TalonFX frontLeftDriveTalonFX = new WPI_TalonFX(RobotMap.frontLeftDriveMotorControllerID);
  static WPI_TalonFX backLeftDriveTalonFX = new WPI_TalonFX(RobotMap.backLeftDriveMotorControllerID);
  static WPI_TalonFX frontRightDriveTalonFX = new WPI_TalonFX(RobotMap.frontRightDriveMotorControllerID);
  static WPI_TalonFX backRightDriveTalonFX = new WPI_TalonFX(RobotMap.backRightDriveMotorControllerID);

  public FalconDriveSubsystem(){
    RobotMap.IAmFalconBot();
	  frontLeftDriveMotorController = frontLeftDriveTalonFX;
	  backLeftDriveMotorController = backLeftDriveTalonFX;
	  frontRightDriveMotorController = frontRightDriveTalonFX;
    backRightDriveMotorController = backRightDriveTalonFX;
    drive = new DifferentialDrive(frontLeftDriveTalonFX, frontRightDriveTalonFX);
  }
 

  //public static DifferentialDrive drive = new DifferentialDrive(frontLeftDriveTalonFX, frontRightDriveTalonFX);
  // No differential or arcade drive for falcons

  @Override
  public void configureEncoders() {
    //empty, because the default for the Falcon is to use the integrated controller
  }

  /**
   * Sets the talons to our preferred defaults
   * We are going away from controller-groups, and back to master-slave
   * Call this in robot-init: it preforms basic setup for ArcadeDrive
   */
  public void resetDriveTrainControllers() {
	  super.resetDriveTrainControllers();
  }

  public void configureDriveTrainControllersForSimpleMagic(){
    super.configureDriveTrainControllersForSimpleMagic();
  } // End configureDriveTrainControllersForSimpleMagic
}
