/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.RobotMap;
import frc.robot.commands.DriveManuallyCommand;

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
	  frontLeftDriveMotorController = frontLeftDriveTalonFX;
	  backLeftDriveMotorController = backLeftDriveTalonFX;
	  frontRightDriveMotorController = frontRightDriveTalonFX;
	  backRightDriveMotorController = backRightDriveTalonFX;
  }
 

  //public static DifferentialDrive drive = new DifferentialDrive(frontLeftDriveTalonFX, frontRightDriveTalonFX);
  // No differential or arcade drive for falcons
  
  public void manualDrive(double move, double turn) {
    frontLeftDriveTalonFX.set(ControlMode.PercentOutput, move, DemandType.ArbitraryFeedForward, +turn);
    frontRightDriveTalonFX.set(ControlMode.PercentOutput, move, DemandType.ArbitraryFeedForward, -turn);
  }

  public void zeroDriveEncoders() {
    frontLeftDriveTalonFX.setSelectedSensorPosition(0);
    frontRightDriveTalonFX.setSelectedSensorPosition(0);
  }

  public int getLeftEncoder() {
    return frontLeftDriveTalonFX.getSelectedSensorPosition();
  }

  public int getRightEncoder() {
    return frontRightDriveTalonFX.getSelectedSensorPosition();
  }

  /**
   * Sets the talons to our preferred defaults
   * We are going away from controller-groups, and back to master-slave
   * Call this in robot-init: it preforms basic setup for ArcadeDrive
   */
  public void resetDriveTrainControllers() {
	//System.out.println("Hit  resetDriveTrainControllers");
    frontLeftDriveTalonFX.configFactoryDefault();
    backLeftDriveTalonFX.configFactoryDefault();
    frontRightDriveTalonFX.configFactoryDefault();
	backRightDriveTalonFX.configFactoryDefault();
	
	//Set all drive motors to brake mode
    frontLeftDriveTalonFX.setNeutralMode(NeutralMode.Brake);
    backLeftDriveTalonFX.setNeutralMode(NeutralMode.Brake);
    frontRightDriveTalonFX.setNeutralMode(NeutralMode.Brake);
    backRightDriveTalonFX.setNeutralMode(NeutralMode.Brake);

	// Set contrllers to Percent output
    frontLeftDriveTalonFX.set(ControlMode.PercentOutput, 0);
    frontRightDriveTalonFX.set(ControlMode.PercentOutput, 0);

    // Set up followers
    backLeftDriveTalonFX.follow(frontLeftDriveTalonFX);
    backRightDriveTalonFX.follow(frontRightDriveTalonFX);

    // Set controller orientation so both sides show green LEDs when drivetrain is going forward  
    frontLeftDriveTalonFX.setInverted(false);
    frontRightDriveTalonFX.setInverted(true);
    backLeftDriveTalonFX.setInverted(InvertType.FollowMaster);
    backRightDriveTalonFX.setInverted(InvertType.FollowMaster);

    // Configure Encoders
    frontLeftDriveTalonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    frontRightDriveTalonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // Set encoder phase so values increase when controller LEDs are green
    frontLeftDriveTalonFX.setSensorPhase(true);
    frontRightDriveTalonFX.setSensorPhase(true);
  }

  // replace with configure controllers for aux closed loop PID when ready
  public void configureDriveTrainControllersForSimpleMagic(){

	// Configure the encoders for PID control
	frontLeftDriveTalonFX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);			
	frontRightDriveTalonFX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID_PRIMARY,	RobotMap.configureTimeoutMs);
	
	/* Set status frame periods to ensure we don't have stale data */
	frontRightDriveTalonFX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 20, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 20, RobotMap.configureTimeoutMs);

	/* Configure motor neutral deadband */
	frontRightDriveTalonFX.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);

    /**
	 * Max out the peak output (for all modes).  
	 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
	 */
	frontLeftDriveTalonFX.configPeakOutputForward(+1.0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.configPeakOutputReverse(-1.0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);

	frontRightDriveTalonFX.configPeakOutputForward(+1.0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configPeakOutputReverse(-1.0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);

	/* FPID Gains for each side of drivetrain */
	frontLeftDriveTalonFX.config_kP(RobotMap.SLOT_0, RobotMap.P_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.config_kI(RobotMap.SLOT_0, RobotMap.I_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.config_kD(RobotMap.SLOT_0, RobotMap.D_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.config_kF(RobotMap.SLOT_0, RobotMap.F_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.config_IntegralZone(RobotMap.SLOT_0, RobotMap.Izone_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.configAllowableClosedloopError(RobotMap.SLOT_0, 0, RobotMap.configureTimeoutMs);

	frontRightDriveTalonFX.config_kP(RobotMap.SLOT_0, RobotMap.P_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_kI(RobotMap.SLOT_0, RobotMap.I_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_kD(RobotMap.SLOT_0, RobotMap.D_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_kF(RobotMap.SLOT_0, RobotMap.F_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_IntegralZone(RobotMap.SLOT_0, RobotMap.Izone_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configAllowableClosedloopError(RobotMap.SLOT_0, 0, RobotMap.configureTimeoutMs);

    /**
	 * 1ms per loop.  PID loop can be slowed down if need be.
	 * For example,
	 * - if sensor updates are too slow
	 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
	 * - sensor movement is very slow causing the derivative error to be near zero.
	 */
	frontRightDriveTalonFX.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);

    /* Motion Magic Configurations */
    /**Need to replace numbers with real measured values for acceleration and cruise vel. */
	frontLeftDriveTalonFX.configMotionAcceleration(RobotMap.acceleration, RobotMap.configureTimeoutMs);
    frontLeftDriveTalonFX.configMotionCruiseVelocity(RobotMap.cruiseVelocity, RobotMap.configureTimeoutMs);
    frontLeftDriveTalonFX.configMotionSCurveStrength(RobotMap.smoothing);

    frontRightDriveTalonFX.configMotionAcceleration(RobotMap.acceleration, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configMotionCruiseVelocity(RobotMap.cruiseVelocity, RobotMap.configureTimeoutMs);
    frontRightDriveTalonFX.configMotionSCurveStrength(RobotMap.smoothing);

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
