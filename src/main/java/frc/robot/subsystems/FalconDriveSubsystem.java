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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ManualDrivingCommand;

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
 
  // encoder positions for aux closed loop PID (driving straight)
  public int getHeadingPosition() {
    return frontRightDriveTalonFX.getSelectedSensorPosition(1);
  }
  public int getDistancePosition() {
    return frontRightDriveTalonFX.getSelectedSensorPosition(0);
  }


  public void DriveTrainCoastMode() {
    frontLeftDriveTalonFX.setNeutralMode(NeutralMode.Coast);
    backLeftDriveTalonFX.setNeutralMode(NeutralMode.Coast);
    frontRightDriveTalonFX.setNeutralMode(NeutralMode.Coast);
    backRightDriveTalonFX.setNeutralMode(NeutralMode.Coast);
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


  public void configureDriveTrainControllersForAuxClosedLoopPID(){
	
	// Configure the left Talon's selected sensor 
	frontLeftDriveTalonFX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);

	/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
	frontRightDriveTalonFX.configRemoteFeedbackFilter(frontLeftDriveTalonFX.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, RobotMap.REMOTE_0,	RobotMap.configureTimeoutMs);
	
	/* Setup Sum signal to be used for Distance */
	frontRightDriveTalonFX.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, RobotMap.configureTimeoutMs);				// Feedback Device of Remote Talon
	frontRightDriveTalonFX.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.configureTimeoutMs);	// Quadrature Encoder of current Talon
	
	/* Setup Difference signal to be used for Turn */
	frontRightDriveTalonFX.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.configureTimeoutMs);
	
	/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
	frontRightDriveTalonFX.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);
	
	/* Scale Feedback by 0.5 to half the sum of Distance */
	frontRightDriveTalonFX.configSelectedFeedbackCoefficient(0.5, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);
	
	/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
	frontRightDriveTalonFX.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, RobotMap.PID_TURN, RobotMap.configureTimeoutMs);
	
	/* Scale the Feedback Sensor using a coefficient */
	frontRightDriveTalonFX.configSelectedFeedbackCoefficient( 1, RobotMap.PID_TURN, RobotMap.configureTimeoutMs);
	
	/* Set status frame periods to ensure we don't have stale data */
	frontRightDriveTalonFX.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.configureTimeoutMs);

	/* Configure motor neutral deadband */
	frontRightDriveTalonFX.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonFX.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);
	
    /* Motion Magic Configurations */
    /**Need to replace numbers with real measured values for acceleration and cruise vel. */
	frontRightDriveTalonFX.configMotionAcceleration(RobotMap.acceleration, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configMotionCruiseVelocity(RobotMap.cruiseVelocity, RobotMap.configureTimeoutMs);

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

	/* FPID Gains for distance servo */
	frontRightDriveTalonFX.config_kP(RobotMap.SLOT_0, RobotMap.P_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_kI(RobotMap.SLOT_0, RobotMap.I_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_kD(RobotMap.SLOT_0, RobotMap.D_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_kF(RobotMap.SLOT_0, RobotMap.F_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_IntegralZone(RobotMap.SLOT_0, RobotMap.Izone_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configAllowableClosedloopError(RobotMap.SLOT_0, 0, RobotMap.configureTimeoutMs);

	/* FPID Gains for turn servo */
	frontRightDriveTalonFX.config_kP(RobotMap.SLOT_1, RobotMap.P_1, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_kI(RobotMap.SLOT_1, RobotMap.I_1, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_kD(RobotMap.SLOT_1, RobotMap.D_1, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_kF(RobotMap.SLOT_1, RobotMap.F_1, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.config_IntegralZone(RobotMap.SLOT_1, RobotMap.Izone_1, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configClosedLoopPeakOutput(RobotMap.SLOT_1, RobotMap.PeakOutput_1, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configAllowableClosedloopError(RobotMap.SLOT_1, 0, RobotMap.configureTimeoutMs);

	/**
	 * 1ms per loop.  PID loop can be slowed down if need be.
	 * For example,
	 * - if sensor updates are too slow
	 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
	 * - sensor movement is very slow causing the derivative error to be near zero.
	 */
	frontRightDriveTalonFX.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.configClosedLoopPeriod(1, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);

	/**
	 * configAuxPIDPolarity(boolean invert, int timeoutMs)
	 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
	 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
	 */
	frontRightDriveTalonFX.configAuxPIDPolarity(false, RobotMap.configureTimeoutMs);
	frontRightDriveTalonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
	frontRightDriveTalonFX.selectProfileSlot(RobotMap.SLOT_0, RobotMap.PID_PRIMARY);
	frontRightDriveTalonFX.selectProfileSlot(RobotMap.SLOT_1, RobotMap.PID_TURN);

  } // End configureDriveTrainControllersForAuxClosedLoopPID



  public void simpleMotionMagicTest(int leftEncoderVal, int rightEncoderVal) {
	// Test method that moves robot forward a given number of wheel rotations  
    frontLeftDriveTalonFX.set(ControlMode.MotionMagic, leftEncoderVal);
	frontRightDriveTalonFX.set(ControlMode.MotionMagic, rightEncoderVal);
  }

  public void differentialMotionMagicTest(int distance, int heading) {
	  frontRightDriveTalonFX.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, heading);
	  frontLeftDriveTalonFX.follow(frontRightDriveTalonFX,FollowerType.AuxOutput1);

  }


  public boolean isOnTarget(int leftEncoderTarget, int rightEncoderTarget){
    return isOnTarget(leftEncoderTarget, rightEncoderTarget,RobotMap.defaultAcceptableError);
  }
  public boolean isOnTarget(int leftEncoderTarget, int rightEncoderTarget, int acceptableError){
    int leftError = Math.abs(leftEncoderTarget - getLeftEncoder());
	int rightError = Math.abs(rightEncoderTarget - getRightEncoder());
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
     setDefaultCommand(new ManualDrivingCommand());
  }
}
