/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveManuallyCommand;

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

  public static DifferentialDrive drive = new DifferentialDrive(frontLeftDriveTalonSRX, frontRightDriveTalonSRX);

  public TalonDriveSubsystem(){
	frontLeftDriveMotorController = frontLeftDriveTalonSRX;
	backLeftDriveMotorController = backLeftDriveTalonSRX;
	frontRightDriveMotorController = frontRightDriveTalonSRX;
	backRightDriveMotorController = backRightDriveTalonSRX;
  }

  public void manualDrive(double move, double turn) {
	drive.arcadeDrive(move, turn);
  }

  public void zeroDriveEncoders() {
    frontLeftDriveTalonSRX.setSelectedSensorPosition(0);
    frontRightDriveTalonSRX.setSelectedSensorPosition(0);
  }

  public int getLeftEncoder() {
    return frontLeftDriveTalonSRX.getSelectedSensorPosition();
  }

  public int getRightEncoder() {
    return frontRightDriveTalonSRX.getSelectedSensorPosition();
  }
  /*
  // encoder positions for aux closed loop PID (driving straight)
  public int getHeadingPosition() {
    return frontRightDriveTalonSRX.getSelectedSensorPosition(1);
  }

  public int getDistancePosition() {
    return frontRightDriveTalonSRX.getSelectedSensorPosition(0);
  }
  */
  public void DriveTrainCoastMode() {
    frontLeftDriveTalonSRX.setNeutralMode(NeutralMode.Coast);
    backLeftDriveTalonSRX.setNeutralMode(NeutralMode.Coast);
    frontRightDriveTalonSRX.setNeutralMode(NeutralMode.Coast);
    backRightDriveTalonSRX.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Sets the talons to our preferred defaults
   * We are going away from controller-groups, and back to master-slave
   * Call this in robot-init: it preforms basic setup for ArcadeDrive
   */
  public void resetDriveTrainControllers() {
    //System.out.println("Hit  resetDriveTrainControllers");
    frontLeftDriveTalonSRX.configFactoryDefault();
    backLeftDriveTalonSRX.configFactoryDefault();
    frontRightDriveTalonSRX.configFactoryDefault();
	backRightDriveTalonSRX.configFactoryDefault();
	
	//Set all drive motors to brake mode
    frontLeftDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    backLeftDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    frontRightDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    backRightDriveTalonSRX.setNeutralMode(NeutralMode.Brake);

	// Set contrllers to Percent output
    frontLeftDriveTalonSRX.set(ControlMode.PercentOutput, 0);
    frontRightDriveTalonSRX.set(ControlMode.PercentOutput, 0);

    // Set up followers
    backLeftDriveTalonSRX.follow(frontLeftDriveTalonSRX);
    backRightDriveTalonSRX.follow(frontRightDriveTalonSRX);

    // Set controller orientation so both sides show green LEDs when drivetrain is going forward  
    frontLeftDriveTalonSRX.setInverted(false);
    frontRightDriveTalonSRX.setInverted(true);
    backLeftDriveTalonSRX.setInverted(InvertType.FollowMaster);
    backRightDriveTalonSRX.setInverted(InvertType.FollowMaster);

    // Configure Encoders
    frontLeftDriveTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    frontRightDriveTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // Set encoder phase so values increase when controller LEDs are green
    frontLeftDriveTalonSRX.setSensorPhase(true);
    frontRightDriveTalonSRX.setSensorPhase(true);
    // Prevent WPI drivetrain class from inverting input for right side motors because we already inverted them
    drive.setRightSideInverted(false);
  }

  // replace with configure controllers for aux closed loop PID when ready
  public void configureDriveTrainControllersForSimpleMagic(){

	// Configure the encoders for PID control
	frontLeftDriveTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);			
	frontRightDriveTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID_PRIMARY,	RobotMap.configureTimeoutMs);
	
	/* Set status frame periods to ensure we don't have stale data */
	frontRightDriveTalonSRX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 20, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 20, RobotMap.configureTimeoutMs);

	/* Configure motor neutral deadband */
	frontRightDriveTalonSRX.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);

    /**
	 * Max out the peak output (for all modes).  
	 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
	 */
	frontLeftDriveTalonSRX.configPeakOutputForward(+1.0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.configPeakOutputReverse(-1.0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);

	frontRightDriveTalonSRX.configPeakOutputForward(+1.0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.configPeakOutputReverse(-1.0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);


	/* FPID Gains for each side of drivetrain */
	frontLeftDriveTalonSRX.config_kP(RobotMap.SLOT_0, RobotMap.P_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.config_kI(RobotMap.SLOT_0, RobotMap.I_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.config_kD(RobotMap.SLOT_0, RobotMap.D_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.config_kF(RobotMap.SLOT_0, RobotMap.F_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.config_IntegralZone(RobotMap.SLOT_0, RobotMap.Izone_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.configAllowableClosedloopError(RobotMap.SLOT_0, 0, RobotMap.configureTimeoutMs);

	frontRightDriveTalonSRX.config_kP(RobotMap.SLOT_0, RobotMap.P_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.config_kI(RobotMap.SLOT_0, RobotMap.I_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.config_kD(RobotMap.SLOT_0, RobotMap.D_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.config_kF(RobotMap.SLOT_0, RobotMap.F_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.config_IntegralZone(RobotMap.SLOT_0, RobotMap.Izone_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.configAllowableClosedloopError(RobotMap.SLOT_0, 0, RobotMap.configureTimeoutMs);

    /**
	 * 1ms per loop.  PID loop can be slowed down if need be.
	 * For example,
	 * - if sensor updates are too slow
	 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
	 * - sensor movement is very slow causing the derivative error to be near zero.
	 */
	frontRightDriveTalonSRX.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);

    /* Motion Magic Configurations */
    /**Need to replace numbers with real measured values for acceleration and cruise vel. */
	frontLeftDriveTalonSRX.configMotionAcceleration(RobotMap.acceleration, RobotMap.configureTimeoutMs);
    frontLeftDriveTalonSRX.configMotionCruiseVelocity(RobotMap.cruiseVelocity, RobotMap.configureTimeoutMs);
    frontLeftDriveTalonSRX.configMotionSCurveStrength(RobotMap.smoothing);

    frontRightDriveTalonSRX.configMotionAcceleration(RobotMap.acceleration, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.configMotionCruiseVelocity(RobotMap.cruiseVelocity, RobotMap.configureTimeoutMs);
    frontRightDriveTalonSRX.configMotionSCurveStrength(RobotMap.smoothing);

  } // End configureDriveTrainControllersForSimpleMagic

  public void simpleMotionMagic(int leftEncoderVal, int rightEncoderVal) {
	// Test method that moves robot forward a given number of wheel rotations  
    frontLeftDriveTalonSRX.set(ControlMode.MotionMagic, leftEncoderVal);
	frontRightDriveTalonSRX.set(ControlMode.MotionMagic, rightEncoderVal);
  }

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
	int distanceError = driveTarget - frontRightDriveTalonSRX.getActiveTrajectoryPosition(0);
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

  public void feed(){
    drive.feed();
  }

  public void driveTrainBrakeMode() {
	frontLeftDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    backLeftDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    frontRightDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    backRightDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void initDefaultCommand() {
	// Set the default command for a subsystem here.
	System.out.println("Default command set");
    setDefaultCommand(new DriveManuallyCommand());
  }
}
