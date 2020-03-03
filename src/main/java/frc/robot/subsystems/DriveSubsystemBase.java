/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public static DifferentialDrive drive;

  DriveSubsystemBase(){
	  super();
    System.out.println("Made a DriveSubsystem");
  }

  public  double deadbandMove(double move) {
    if (Math.abs(move) >= RobotMap.deadbandY){
      if (move > 0) {
        move = (move - RobotMap.deadbandY) /(1-RobotMap.deadbandY); 
      }
      else{
        move = (move + RobotMap.deadbandY) /(1-RobotMap.deadbandY);  
      }
    }
    else {
      move = 0;
    } 
    return move;
  }
  
public  double deadbandTurn(double turn) {
  if (Math.abs(turn) >= RobotMap.deadbandX){
    if (turn > 0) {
      turn = (turn - RobotMap.deadbandX) /(1-RobotMap.deadbandX); 
    }
    else{
      turn = (turn + RobotMap.deadbandX) /(1-RobotMap.deadbandX);  
    }
  }
  else {
    turn = 0;
  } 
  return turn;
}

  public void manualDrive(double move, double turn) {
	  drive.arcadeDrive(deadbandMove(move), deadbandTurn(turn));
  }




  public void zeroDriveEncoders() {
    frontLeftDriveMotorController.setSelectedSensorPosition(0);
    frontRightDriveMotorController.setSelectedSensorPosition(0);
    frontLeftDriveMotorController.setNeutralMode(NeutralMode.Coast);
    backLeftDriveMotorController.setNeutralMode(NeutralMode.Coast);
    frontRightDriveMotorController.setNeutralMode(NeutralMode.Coast);
    backRightDriveMotorController.setNeutralMode(NeutralMode.Coast);  
  }

  public int getLeftEncoder() {
    return frontLeftDriveMotorController.getSelectedSensorPosition();
  }

  public int getLeftEncoderSpeed(){
    return frontLeftDriveMotorController.getSelectedSensorVelocity();
  }

  public int getRightEncoderSpeed(){
    return frontRightDriveMotorController.getSelectedSensorVelocity();
  }
  public int getRightEncoder() {
    return frontRightDriveMotorController.getSelectedSensorPosition();
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
    frontLeftDriveMotorController.configFactoryDefault();
    backLeftDriveMotorController.configFactoryDefault();
    frontRightDriveMotorController.configFactoryDefault();
    backRightDriveMotorController.configFactoryDefault();
    
    configureEncoders();
	
	  //Set all drive motors to brake mode
    frontLeftDriveMotorController.setNeutralMode(NeutralMode.Brake);
    backLeftDriveMotorController.setNeutralMode(NeutralMode.Brake);
    frontRightDriveMotorController.setNeutralMode(NeutralMode.Brake);
    backRightDriveMotorController.setNeutralMode(NeutralMode.Brake);

  	// Set controllers to Percent output
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

    // Set encoder phase so values increase when controller LEDs are green
    frontLeftDriveMotorController.setSensorPhase(true);
    frontRightDriveMotorController.setSensorPhase(true);
    // Prevent WPI drivetrain class from inverting input for right side motors because we already inverted them
    drive.setRightSideInverted(false);
  }

  /**
   * This had to be factored out because the Falcon has a different sensor than
   * is used by a TalonBot.
   */
  public abstract void configureEncoders();

  public void configureDriveTrainControllersForSimpleMagic(){
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

  public void simpleMotionMagic(int leftEncoderVal, int rightEncoderVal) {
	// Test method that moves robot forward a given number of wheel rotations  
    frontLeftDriveMotorController.set(ControlMode.MotionMagic, leftEncoderVal);
	  frontRightDriveMotorController.set(ControlMode.MotionMagic, rightEncoderVal);
  }

  public boolean isOnTarget(int leftEncoderTarget, int rightEncoderTarget){
	  // stuff parameters and call again (-200 is an impossible heading)
    return isOnTarget(leftEncoderTarget, rightEncoderTarget, RobotMap.defaultAcceptableError, -200);
  }

  public boolean isOnTarget(int leftEncoderTarget, int rightEncoderTarget, int acceptableError){
    // stuff parameters and call again (-200 is an impossible heading)
    int leftError = Math.abs(leftEncoderTarget - getLeftEncoder());
	  int rightError = Math.abs(rightEncoderTarget - getRightEncoder());
	  SmartDashboard.putNumber("Error L", leftError);
	  SmartDashboard.putNumber("Error R", rightError);
    if(leftError <= acceptableError && rightError <= acceptableError){
		  if(wasOnTarget){
        return true;
      }
		  wasOnTarget = true;//Dont return true if we just 
	  }
	  else{
		  wasOnTarget=false;
	  }
    return false;
  }
  
  public boolean isOnTarget(int leftEncoderTarget, int rightEncoderTarget, int acceptableError, double targetHeading){
    return isOnTarget(leftEncoderTarget, rightEncoderTarget, acceptableError);
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

  public void feed(){
    drive.feed();
  }

  public void driveTrainBrakeMode() {
	  frontLeftDriveMotorController.setNeutralMode(NeutralMode.Brake);
    backLeftDriveMotorController.setNeutralMode(NeutralMode.Brake);
    frontRightDriveMotorController.setNeutralMode(NeutralMode.Brake);
    backRightDriveMotorController.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveManuallyCommand());
  }
}
