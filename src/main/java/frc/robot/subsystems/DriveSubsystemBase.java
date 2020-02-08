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
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveManuallyCommand;

/**
 * Add your docs here. TODO: Add docs
 */
public abstract class DriveSubsystemBase extends Subsystem {
  // Put methods for controlling this subsystem here. Call these from Commands.

  //For isOnTarget
  boolean wasOnTarget = false;
  int withinAcceptableErrorLoops = 0;


  static BaseTalon frontLeftDriveMotorController;
  static BaseTalon backLeftDriveMotorController;
  static BaseTalon frontRightDriveMotorController;
  static BaseTalon backRightDriveMotorController;

  DriveSubsystemBase(){
	  super();
	  System.out.println("Made a DriveSubsystem");
  }
  public abstract void manualDrive(double move, double turn);

  public void zeroDriveEncoders() {
    frontLeftDriveMotorController.setSelectedSensorPosition(0);
    frontRightDriveMotorController.setSelectedSensorPosition(0);
  }

  public int getLeftEncoder() {
    return frontLeftDriveMotorController.getSelectedSensorPosition();
  }

  public int getRightEncoder() {
    return frontRightDriveMotorController.getSelectedSensorPosition();
  }
 
  // encoder positions for aux closed loop PID (driving straight)
  public int getHeadingPosition() {
    return frontRightDriveMotorController.getSelectedSensorPosition(1);
  }
  
  public int getDistancePosition() {
    return frontRightDriveMotorController.getSelectedSensorPosition(0);
  }


  public void DriveTrainCoastMode() {
    frontLeftDriveMotorController.setNeutralMode(NeutralMode.Coast);
    backLeftDriveMotorController.setNeutralMode(NeutralMode.Coast);
    frontRightDriveMotorController.setNeutralMode(NeutralMode.Coast);
    backRightDriveMotorController.setNeutralMode(NeutralMode.Coast);
  }
  /**
   * Sets the talons to our preferred defaults
   * We are going away from controller-groups, and back to master-slave
   * Call this in robot-init: it preforms basic setup for ArcadeDrive
   */
  public void resetDriveTrainControllers() {
	//System.out.println("Hit  resetDriveTrainControllers");
    frontLeftDriveMotorController.configFactoryDefault();
    backLeftDriveMotorController.configFactoryDefault();
    frontRightDriveMotorController.configFactoryDefault();
	backRightDriveMotorController.configFactoryDefault();
	
	//Set all drive motors to brake mode
    frontLeftDriveMotorController.setNeutralMode(NeutralMode.Brake);
    backLeftDriveMotorController.setNeutralMode(NeutralMode.Brake);
    frontRightDriveMotorController.setNeutralMode(NeutralMode.Brake);
    backRightDriveMotorController.setNeutralMode(NeutralMode.Brake);

	// Set contrllers to Percent output
    frontLeftDriveMotorController.set(ControlMode.PercentOutput, 0);
    frontRightDriveMotorController.set(ControlMode.PercentOutput, 0);


    // Set up followers
    backLeftDriveMotorController.follow(frontLeftDriveMotorController);
    backRightDriveMotorController.follow(frontRightDriveMotorController);

    // Set controller orientation so both sides show green LEDs when drivetrain is going forward  
    frontLeftDriveMotorController.setInverted(false);
    frontRightDriveMotorController.setInverted(true);
    backLeftDriveMotorController.setInverted(InvertType.FollowMaster);
    backRightDriveMotorController.setInverted(InvertType.FollowMaster);

    // Configure Encoders
    frontLeftDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    frontRightDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // Set encoder phase so values increase when controller LEDs are green
    frontLeftDriveMotorController.setSensorPhase(true);
    frontRightDriveMotorController.setSensorPhase(true);
  }
  // replace with configure controllers for aux closed loop PID when ready
  public void configureDriveTrainControllersForSimpleMagic(){

	// Configure the encoders for PID control
	frontLeftDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);			
	frontRightDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID_PRIMARY,	RobotMap.configureTimeoutMs);
	
	/* Set status frame periods to ensure we don't have stale data */
	frontRightDriveMotorController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 20, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 20, RobotMap.configureTimeoutMs);

	/* Configure motor neutral deadband */
	frontRightDriveMotorController.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);

    /**
	 * Max out the peak output (for all modes).  
	 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
	 */
	frontLeftDriveMotorController.configPeakOutputForward(+1.0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.configPeakOutputReverse(-1.0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);

	frontRightDriveMotorController.configPeakOutputForward(+1.0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configPeakOutputReverse(-1.0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);


	/* FPID Gains for each side of drivetrain */
	frontLeftDriveMotorController.config_kP(RobotMap.SLOT_0, RobotMap.P_0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.config_kI(RobotMap.SLOT_0, RobotMap.I_0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.config_kD(RobotMap.SLOT_0, RobotMap.D_0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.config_kF(RobotMap.SLOT_0, RobotMap.F_0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.config_IntegralZone(RobotMap.SLOT_0, RobotMap.Izone_0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.configAllowableClosedloopError(RobotMap.SLOT_0, 0, RobotMap.configureTimeoutMs);

	frontRightDriveMotorController.config_kP(RobotMap.SLOT_0, RobotMap.P_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_kI(RobotMap.SLOT_0, RobotMap.I_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_kD(RobotMap.SLOT_0, RobotMap.D_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_kF(RobotMap.SLOT_0, RobotMap.F_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_IntegralZone(RobotMap.SLOT_0, RobotMap.Izone_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configAllowableClosedloopError(RobotMap.SLOT_0, 0, RobotMap.configureTimeoutMs);

    /**
	 * 1ms per loop.  PID loop can be slowed down if need be.
	 * For example,
	 * - if sensor updates are too slow
	 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
	 * - sensor movement is very slow causing the derivative error to be near zero.
	 */
	frontRightDriveMotorController.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);

    /* Motion Magic Configurations */
    /**Need to replace numbers with real measured values for acceleration and cruise vel. */
	frontLeftDriveMotorController.configMotionAcceleration(RobotMap.acceleration, RobotMap.configureTimeoutMs);
    frontLeftDriveMotorController.configMotionCruiseVelocity(RobotMap.cruiseVelocity, RobotMap.configureTimeoutMs);
    frontLeftDriveMotorController.configMotionSCurveStrength(RobotMap.smoothing);

    frontRightDriveMotorController.configMotionAcceleration(RobotMap.acceleration, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configMotionCruiseVelocity(RobotMap.cruiseVelocity, RobotMap.configureTimeoutMs);
    frontRightDriveMotorController.configMotionSCurveStrength(RobotMap.smoothing);

  } // End configureDriveTrainControllersForSimpleMagic


  public void configureDriveTrainControllersForAuxClosedLoopPID(){
	
	// Configure the left Talon's selected sensor 
	frontLeftDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);

	/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
	frontRightDriveMotorController.configRemoteFeedbackFilter(frontLeftDriveMotorController.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_0,	RobotMap.configureTimeoutMs);
	
	/* Setup Sum signal to be used for Distance */
	frontRightDriveMotorController.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, RobotMap.configureTimeoutMs);				// Feedback Device of Remote Talon
	frontRightDriveMotorController.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.configureTimeoutMs);	// Quadrature Encoder of current Talon
	
	/* Setup Difference signal to be used for Turn */
	frontRightDriveMotorController.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.configureTimeoutMs);
	
	/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
	frontRightDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);
	
	/* Scale Feedback by 0.5 to half the sum of Distance */
	frontRightDriveMotorController.configSelectedFeedbackCoefficient(0.5, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);
	
	/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
	frontRightDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, RobotMap.PID_TURN, RobotMap.configureTimeoutMs);
	
	/* Scale the Feedback Sensor using a coefficient */
	frontRightDriveMotorController.configSelectedFeedbackCoefficient( 1, RobotMap.PID_TURN, RobotMap.configureTimeoutMs);
	
	/* Set status frame periods to ensure we don't have stale data */
	frontRightDriveMotorController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.configureTimeoutMs);

	/* Configure motor neutral deadband */
	frontRightDriveMotorController.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);
	
    /* Motion Magic Configurations */
    /**Need to replace numbers with real measured values for acceleration and cruise vel. */
	frontRightDriveMotorController.configMotionAcceleration(RobotMap.acceleration, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configMotionCruiseVelocity(RobotMap.cruiseVelocity, RobotMap.configureTimeoutMs);

	/**
	 * Max out the peak output (for all modes).  
	 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
	 */
	frontLeftDriveMotorController.configPeakOutputForward(+1.0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.configPeakOutputReverse(-1.0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
	frontLeftDriveMotorController.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);

	frontRightDriveMotorController.configPeakOutputForward(+1.0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configPeakOutputReverse(-1.0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);

	/* FPID Gains for distance servo */
	frontRightDriveMotorController.config_kP(RobotMap.SLOT_0, RobotMap.P_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_kI(RobotMap.SLOT_0, RobotMap.I_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_kD(RobotMap.SLOT_0, RobotMap.D_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_kF(RobotMap.SLOT_0, RobotMap.F_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_IntegralZone(RobotMap.SLOT_0, RobotMap.Izone_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configAllowableClosedloopError(RobotMap.SLOT_0, 0, RobotMap.configureTimeoutMs);

	/* FPID Gains for turn servo */
	frontRightDriveMotorController.config_kP(RobotMap.SLOT_1, RobotMap.P_1, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_kI(RobotMap.SLOT_1, RobotMap.I_1, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_kD(RobotMap.SLOT_1, RobotMap.D_1, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_kF(RobotMap.SLOT_1, RobotMap.F_1, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.config_IntegralZone(RobotMap.SLOT_1, RobotMap.Izone_1, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configClosedLoopPeakOutput(RobotMap.SLOT_1, RobotMap.PeakOutput_1, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configAllowableClosedloopError(RobotMap.SLOT_1, 0, RobotMap.configureTimeoutMs);

	/**
	 * 1ms per loop.  PID loop can be slowed down if need be.
	 * For example,
	 * - if sensor updates are too slow
	 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
	 * - sensor movement is very slow causing the derivative error to be near zero.
	 */
	frontRightDriveMotorController.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.configClosedLoopPeriod(1, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);

	/**
	 * configAuxPIDPolarity(boolean invert, int timeoutMs)
	 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
	 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
	 */
	frontRightDriveMotorController.configAuxPIDPolarity(false, RobotMap.configureTimeoutMs);
	frontRightDriveMotorController.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
	frontRightDriveMotorController.selectProfileSlot(RobotMap.SLOT_0, RobotMap.PID_PRIMARY);
	frontRightDriveMotorController.selectProfileSlot(RobotMap.SLOT_1, RobotMap.PID_TURN);

  } // End configureDriveTrainControllersForAuxClosedLoopPID



  public void simpleMotionMagicTest(int leftEncoderVal, int rightEncoderVal) {
	// Test method that moves robot forward a given number of wheel rotations  
    frontLeftDriveMotorController.set(ControlMode.MotionMagic, leftEncoderVal);
	frontRightDriveMotorController.set(ControlMode.MotionMagic, rightEncoderVal);
  }

  public void differentialMotionMagicTest(int distance, int heading) {
	  frontRightDriveMotorController.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, heading);
	  frontLeftDriveMotorController.follow(frontRightDriveMotorController,FollowerType.AuxOutput1);

  }


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
  
  public boolean isOnTargetMagicMotion(int driveTarget, int acceptableError){
	int distanceError = driveTarget - frontRightDriveMotorController.getActiveTrajectoryPosition(0);
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
	frontLeftDriveMotorController.setNeutralMode(NeutralMode.Brake);
    backLeftDriveMotorController.setNeutralMode(NeutralMode.Brake);
    frontRightDriveMotorController.setNeutralMode(NeutralMode.Brake);
    backRightDriveMotorController.setNeutralMode(NeutralMode.Brake);
  }
  public void IAmFalconBot() {
    // How many encoder clicks per revolution (change to 2048 for falcon 500
    // encoders)
    RobotMap.encoderUnitsPerShaftRotation = 2048;
    // The difference between the left and right side encoder values when the robot
    // is rotated 180 degrees
    RobotMap.encoderUnitsPerRobotRotation = 3755;// thats the SUM of the two (this is just a rough guess)
    //these values are just guesses at the moment
    RobotMap.cruiseVelocity = 2250;
    RobotMap.acceleration = 2250;
    // Allowable error to exit movement methods
    RobotMap.defaultAcceptableError = 500;
    System.out.println("I AM FALCONBOT! CACAW! CACAAAAAWWWWW!");
}


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveManuallyCommand());
  }
}
