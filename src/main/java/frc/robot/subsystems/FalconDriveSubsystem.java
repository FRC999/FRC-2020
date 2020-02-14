/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveManuallyCommand;

/**
 * Add your docs here.
 */
public class FalconDriveSubsystem extends DriveSubsystemBase {
  //For isOnTarget
  boolean wasOnTarget = false;
  int withinAcceptableErrorLoops = 0;

  static WPI_TalonFX frontLeftDriveTalonFX = new WPI_TalonFX(RobotMap.frontLeftDriveMotorController);
  static WPI_TalonFX backLeftDriveTalonFX = new WPI_TalonFX(RobotMap.backLeftDriveMotorController);
  static WPI_TalonFX frontRightDriveTalonFX = new WPI_TalonFX(RobotMap.frontRightDriveMotorController);
  static WPI_TalonFX backRightDriveTalonFX = new WPI_TalonFX(RobotMap.backRightDriveMotorController);

  public FalconDriveSubsystem(){
    IAmFalconBot();
	  frontLeftDriveMotorController = frontLeftDriveTalonFX;
	  backLeftDriveMotorController = backLeftDriveTalonFX;
	  frontRightDriveMotorController = frontRightDriveTalonFX;
    backRightDriveMotorController = backRightDriveTalonFX;
    drive = new DifferentialDrive(frontLeftDriveTalonFX, frontRightDriveTalonFX);
  }

  //public static DifferentialDrive drive = new DifferentialDrive(frontLeftDriveTalonFX, frontRightDriveTalonFX);
  // No differential or arcade drive for falcons
  
  public void manualDrive(double move, double turn) {
    frontLeftDriveTalonFX.set(ControlMode.PercentOutput, move, DemandType.ArbitraryFeedForward, +turn);
    frontRightDriveTalonFX.set(ControlMode.PercentOutput, move, DemandType.ArbitraryFeedForward, -turn);
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

  /*
  public boolean isOnTarget(int leftEncoderTarget, int rightEncoderTarget){
	// stuff parameters and call again (-200 is an impossible heading)
    return isOnTarget(leftEncoderTarget, rightEncoderTarget, RobotMap.defaultAcceptableError, -200);
  }

  public boolean isOnTarget(int leftEncoderTarget, int rightEncoderTarget, int acceptableError){
    // stuff parameters and call again (-200 is an impossible heading)
    return isOnTarget(leftEncoderTarget, rightEncoderTarget, acceptableError, -200);
  }

  public boolean isOnTarget(int leftEncoderTarget, int rightEncoderTarget, int acceptableError, double targetHeading){
    int leftError = Math.abs(leftEncoderTarget - getLeftEncoder());
    int rightError = Math.abs(rightEncoderTarget - getRightEncoder());
    if (targetHeading != -200){
	  double headingError = Math.abs(Robot.navXSubsystem.getYaw()) - Math.abs(targetHeading);
	  //just show angle error for now to get an idea of if thee is an issue.
	  SmartDashboard.putNumber("Error Heading", headingError);
    }
    SmartDashboard.putNumber("Error L", leftError);
    SmartDashboard.putNumber("Error R", rightError);
    if(leftError <= acceptableError && rightError <= acceptableError){
	  if(wasOnTarget){return true;};
	  wasOnTarget = true;//Dont return true if we just 
    }
    else{
	  wasOnTarget=false;
    }
    return false;
  }
  */
  
  public boolean isOnTargetMagicMotion(int driveTarget, int acceptableError){
	int distanceError = driveTarget - frontRightDriveTalonFX.getActiveTrajectoryPosition(0);
	if (distanceError < +acceptableError && distanceError > -acceptableError) {

		++withinAcceptableErrorLoops;
	} else {
		withinAcceptableErrorLoops = 0;
	}
	if (withinAcceptableErrorLoops > 10){
		return true;
	} else {
		return false;
	}
  }

 
  public void driveTrainBrakeMode() {
	frontLeftDriveTalonFX.setNeutralMode(NeutralMode.Brake);
    backLeftDriveTalonFX.setNeutralMode(NeutralMode.Brake);
    frontRightDriveTalonFX.setNeutralMode(NeutralMode.Brake);
    backRightDriveTalonFX.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void initDefaultCommand() {
    
    // Set the default command for a subsystem here.
     setDefaultCommand(new DriveManuallyCommand());
  }
}
