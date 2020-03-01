/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class DriveEachSideCommand extends Command {
 
  public int l;
  public int r;

  public DriveEachSideCommand(int left, int right) {
    l = left;
    r = right;
    requires(Robot.driveSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    int lEncoder = Robot.driveSubsystem.getLeftEncoder();
    int rEncoder = Robot.driveSubsystem.getRightEncoder();
    l = l + lEncoder;
    r = r + rEncoder;
    Robot.driveSubsystem.simpleMotionMagic(l, r);
    
    SmartDashboard.putNumber("leftTarget", l);
    SmartDashboard.putNumber("RightTarget", r);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.smartDashboardSubsystem.updateEncoderValue();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveSubsystem.isOnTarget(l,r,300);
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
