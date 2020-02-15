/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ShootManuallyCommand extends Command {
  public ShootManuallyCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.shooterSubsystem);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //set shooter wheel to full speed
    Robot.shooterSubsystem.shoot(1);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //use the twist and throttle to control shooter pan and tilt
    double pan = Robot.oi.leftJoystick.getZ();
    double tilt = Robot.oi.leftJoystick.getThrottle();
    Robot.shooterSubsystem.pan(pan);
    Robot.shooterSubsystem.tilt(tilt);
    
    Robot.smartDashboardSubsystem.updateShooterValues();
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
