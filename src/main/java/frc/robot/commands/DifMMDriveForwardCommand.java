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
import frc.robot.RobotMap;

public class DifMMDriveForwardCommand extends Command {
  private static int driveDistance;
  private static int driveTarget;

  public DifMMDriveForwardCommand(int distance) {
    requires(Robot.driveSubsystem);
    driveDistance = distance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //System.out.println("Called initialize");
    //Robot.driveSubsystem.driveTrainBrakeMode();
    int heading = Robot.driveSubsystem.getHeadingPosition();
    int position = Robot.driveSubsystem.getDistancePosition();
    int driveTarget =  driveDistance + position;
    Robot.driveSubsystem.differentialMotionMagicTest(driveTarget, heading);
  }
    

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // check to see if closed loop PID is settled on target
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveSubsystem.isOnTargetMagicMotion(driveTarget, RobotMap.defaultAcceptableError);
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
