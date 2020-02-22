/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveFollowWallCommand extends Command {

  // get target distance from wall in mm
  private static final double targetDistance = RobotMap.distanceFromWall;
  private final AnalogInput ultrasonicLeft = new AnalogInput(RobotMap.ultrasonicInputChannelLeft);
  // use the median of a sliding window of 5 ultrasonic readings to smooth readings 
  private final MedianFilter filteredUltrasonic = new MedianFilter(5);
  private final edu.wpi.first.wpilibj.controller.PIDController UltasonicPID = new edu.wpi.first.wpilibj.controller.PIDController(RobotMap.P_U, RobotMap.I_U, RobotMap.D_U, 0.02);

  public DriveFollowWallCommand() {
    // ***  For the side parameter, True = Left, False = Right   Change to enum ***    
    requires(Robot.driveSubsystem);
  }
  

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    UltasonicPID.setSetpoint(targetDistance);
    UltasonicPID.setTolerance(RobotMap.distanceFromWallTolerance);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Read ultrasonic sensor and get filtered value
    double filteredDistance = filteredUltrasonic.calculate(ultrasonicLeft.getVoltage()) * RobotMap.ultrasonicValueToMMConversionFactor;
    // calculate PID value
    double PIDOutput = UltasonicPID.calculate(filteredDistance);
    // read joystick
    double move = Robot.oi.leftJoystick.getY() * -1; // inverts sign for Y axis
    
    Robot.driveSubsystem.manualDrive(move, PIDOutput);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
