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
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ManualDrivingCommand;

/**
 * Add your docs here. TODO: Add docs
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem here. Call these from Commands.

  static WPI_TalonSRX frontLeftDriveTalonSRX = new WPI_TalonSRX(RobotMap.frontLeftDriveMotorController);
  static WPI_TalonSRX backLeftDriveTalonSRX = new WPI_TalonSRX(RobotMap.backLeftDriveMotorController);
  static WPI_TalonSRX frontRightDriveTalonSRX = new WPI_TalonSRX(RobotMap.frontRightDriveMotorController);
  static WPI_TalonSRX backRightDriveTalonSRX = new WPI_TalonSRX(RobotMap.backRightDriveMotorController);

  public static DifferentialDrive drive = new DifferentialDrive(frontLeftDriveTalonSRX, frontRightDriveTalonSRX);

  public void ManualDrive(double move, double turn) {
    drive.arcadeDrive(move, turn);
    updateEncodersDisplay();
  }

  public void ZeroDriveEncoders() {
    frontLeftDriveTalonSRX.setSelectedSensorPosition(0);
    frontRightDriveTalonSRX.setSelectedSensorPosition(0);
  }

  public int getLeftEncoder() {
    return frontLeftDriveTalonSRX.getSelectedSensorPosition();
  }

  public int getRightEncoder() {
    return frontRightDriveTalonSRX.getSelectedSensorPosition();
  }

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
	System.out.println("Hit  resetDriveTrainControllers");
    frontLeftDriveTalonSRX.configFactoryDefault();
    backLeftDriveTalonSRX.configFactoryDefault();
    frontRightDriveTalonSRX.configFactoryDefault();
    backRightDriveTalonSRX.configFactoryDefault();

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

  /**
   * Self explanatory:
   * Calls resetDriveTrainEncoders, just for safety
   */
  public void configureDriveTrainControllersForSimpleMagic(){
	//resetDriveTrainControllers(); //just to be safe -CMM
	System.out.println("Hit  configureDriveTrainControllersForSimpleMagic");

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

  /**
   * WORK IN PROGRESS, DO NOT USE
   */
  public void configureDriveTrainControllersForDifferentialMagic(){

	// ----- Still a work in progress, do not use ----
	
	// Configure the left Talon's selected sensor 
	frontLeftDriveTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);

	/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
	frontRightDriveTalonSRX.configRemoteFeedbackFilter(frontLeftDriveTalonSRX.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_0,	RobotMap.configureTimeoutMs);
	
	/* Setup Sum signal to be used for Distance */
	frontRightDriveTalonSRX.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, RobotMap.configureTimeoutMs);				// Feedback Device of Remote Talon
	frontRightDriveTalonSRX.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.configureTimeoutMs);	// Quadrature Encoder of current Talon
	
	/* Setup Difference signal to be used for Turn */
	frontRightDriveTalonSRX.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.configureTimeoutMs);
	
	/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
	frontRightDriveTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);
	
	/* Scale Feedback by 0.5 to half the sum of Distance */
	frontRightDriveTalonSRX.configSelectedFeedbackCoefficient(0.5, RobotMap.PID_PRIMARY, RobotMap.configureTimeoutMs);
	
	/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
	frontRightDriveTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, RobotMap.PID_TURN, RobotMap.configureTimeoutMs);
	
	/* Scale the Feedback Sensor using a coefficient */
	frontRightDriveTalonSRX.configSelectedFeedbackCoefficient( 1, RobotMap.PID_TURN, RobotMap.configureTimeoutMs);
	
	/* Set status frame periods to ensure we don't have stale data */
	frontRightDriveTalonSRX.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.configureTimeoutMs);
	frontRightDriveTalonSRX.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.configureTimeoutMs);

	/* Configure motor neutral deadband */
	frontRightDriveTalonSRX.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);
	frontLeftDriveTalonSRX.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);
	
    /* Motion Magic Configurations */
    /**Need to replace numbers with real measured values for acceleration and cruise vel. */
		frontRightDriveTalonSRX.configMotionAcceleration(RobotMap.acceleration, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.configMotionCruiseVelocity(RobotMap.cruiseVelocity, RobotMap.configureTimeoutMs);

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

		/* FPID Gains for distance servo */
		frontRightDriveTalonSRX.config_kP(RobotMap.SLOT_0, RobotMap.P_0, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.config_kI(RobotMap.SLOT_0, RobotMap.I_0, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.config_kD(RobotMap.SLOT_0, RobotMap.D_0, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.config_kF(RobotMap.SLOT_0, RobotMap.F_0, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.config_IntegralZone(RobotMap.SLOT_0, RobotMap.Izone_0, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.configAllowableClosedloopError(RobotMap.SLOT_0, 0, RobotMap.configureTimeoutMs);

		/* FPID Gains for turn servo */
		frontRightDriveTalonSRX.config_kP(RobotMap.SLOT_1, RobotMap.P_1, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.config_kI(RobotMap.SLOT_1, RobotMap.I_1, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.config_kD(RobotMap.SLOT_1, RobotMap.D_1, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.config_kF(RobotMap.SLOT_1, RobotMap.F_1, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.config_IntegralZone(RobotMap.SLOT_1, RobotMap.Izone_1, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.configClosedLoopPeakOutput(RobotMap.SLOT_1, RobotMap.PeakOutput_1, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.configAllowableClosedloopError(RobotMap.SLOT_1, 0, RobotMap.configureTimeoutMs);

		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		frontRightDriveTalonSRX.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);
		frontRightDriveTalonSRX.configClosedLoopPeriod(1, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);

		/**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		frontRightDriveTalonSRX.configAuxPIDPolarity(false, RobotMap.configureTimeoutMs);

  } // End configureDriveTrainControllersForDifferentialMagic

  public void SimpleMotionMagicTest(int leftEncoderVal, int rightEncoderVal) {
	// Test method that moves robot forward a given number of wheel rotations  
    //int leftTargetEncoderVal = 4096 * leftWheelTurns;
	//System.out.println("Hit WertzCode");
	//int rightTargetEncoderVal = 4096 * rightWheelTurns;
    frontLeftDriveTalonSRX.set(ControlMode.MotionMagic, leftEncoderVal);
	frontRightDriveTalonSRX.set(ControlMode.MotionMagic, rightEncoderVal);
  }

  public void DriveTrainBrakeMode() {
    frontLeftDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    backLeftDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    frontRightDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    backRightDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
  }

  public void updateEncodersDisplay(){
    SmartDashboard.putNumber("left encoder", getLeftEncoder());
    SmartDashboard.putNumber("right encoder", getRightEncoder());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualDrivingCommand());
  }
}
