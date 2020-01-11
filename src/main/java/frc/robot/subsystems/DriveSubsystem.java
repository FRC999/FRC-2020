/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.ManualDrivingCommand;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
static WPI_TalonSRX frontLeftDriveTalonSRX= new WPI_TalonSRX(RobotMap.frontLeftDriveMotorController);
static WPI_TalonSRX backLeftDriveTalonSRX= new WPI_TalonSRX(RobotMap.backLeftDriveMotorController);
static WPI_TalonSRX frontRightDriveTalonSRX= new WPI_TalonSRX(RobotMap.frontRightDriveMotorController);
static WPI_TalonSRX backRightDriveTalonSRX= new WPI_TalonSRX(RobotMap.backRightDriveMotorController);

static SpeedControllerGroup leftDriveMotorGroup = new SpeedControllerGroup(frontLeftDriveTalonSRX, backLeftDriveTalonSRX);
static SpeedControllerGroup rightDriveMotorGroup = new SpeedControllerGroup(frontRightDriveTalonSRX, backRightDriveTalonSRX);

public static DifferentialDrive drive = new DifferentialDrive(leftDriveMotorGroup, rightDriveMotorGroup);

public void ManualDrive(double move, double turn){
 
 /*
  // Testing Mode
  //Slow things down for testing mode.... Comment out when not testing
  if (move > 0.25) {
    move = 0.25;
  }
  if (turn > 0.25) {
    turn = 0.25;
  }
  // End Testing Mode
*/

  drive.arcadeDrive(move, turn);
}

public void ZeroDriveEncoders(){ 
 frontLeftDriveTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
 frontLeftDriveTalonSRX.setSelectedSensorPosition(0);
 frontRightDriveTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
 frontRightDriveTalonSRX.setSelectedSensorPosition(0);
}

public void DriveTrainCoastMode(){ 
  frontLeftDriveTalonSRX.setNeutralMode(NeutralMode.Coast);  
  backLeftDriveTalonSRX.setNeutralMode(NeutralMode.Coast);
  frontRightDriveTalonSRX.setNeutralMode(NeutralMode.Coast);
  backRightDriveTalonSRX.setNeutralMode(NeutralMode.Coast);
}

public void ResetDriveTrainControllers(){
  frontLeftDriveTalonSRX.configFactoryDefault();
  backLeftDriveTalonSRX.configFactoryDefault();
  frontRightDriveTalonSRX.configFactoryDefault();
  backRightDriveTalonSRX.configFactoryDefault();
}

public void DriveTrainBrakeMode(){
  frontLeftDriveTalonSRX.setNeutralMode(NeutralMode.Brake);  
  backLeftDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
  frontRightDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
  backRightDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualDrivingCommand());
  }
}
