/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ShooterTurretCenterCommand extends Command {
  /**for now, would only work within a 180 degree range; needs testing */
  public ShooterTurretCenterCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.shooterSubsystem);
  }

   boolean canFinish;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    canFinish = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   Robot.shooterSubsystem.panToRobotFront();
   // {canFinish = true;}
    /*else if (Robot.shooterSubsystem.getPanEncoder() > 3340) // outside of range where encoders are clear
    {Robot.shooterSubsystem.pan(0.4);}*/
    


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
