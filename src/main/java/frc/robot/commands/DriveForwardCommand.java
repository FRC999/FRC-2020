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

public class DriveForwardCommand extends Command {
  private static final int driveDistance = 49700;
   
  int rightTarget;
  int leftTarget;
  public DriveForwardCommand() {
    requires(Robot.driveSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Called initialize");
    leftTarget = Robot.driveSubsystem.getLeftEncoder() + driveDistance;
    rightTarget = Robot.driveSubsystem.getRightEncoder() + driveDistance;
    Robot.driveSubsystem.simpleMotionMagicTest(50000, 50000);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveSubsystem.feed();
    Robot.smartDashboardSubsystem.updateEncoderValue();
    SmartDashboard.putNumber("leftTarget",leftTarget);
    SmartDashboard.putNumber("RightTarget", rightTarget);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveSubsystem.isOnTarget(leftTarget,rightTarget,300);
    
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("ENDED DRIVEFORWARD");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
