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

public class DriveForwardCommand extends Command {
  private static int driveDistance;
   
  int rightTarget;
  int leftTarget;

  public DriveForwardCommand(int distance) {
    requires(Robot.driveSubsystem);
    driveDistance = distance;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Called initialize");
    Robot.driveSubsystem.driveTrainBrakeMode();
    int lEncoder = Robot.driveSubsystem.getLeftEncoder();
    int rEncoder = Robot.driveSubsystem.getRightEncoder();
    System.out.println(lEncoder);
    System.out.println(rEncoder);
    leftTarget =  driveDistance + lEncoder;
    rightTarget = driveDistance + rEncoder;
    Robot.driveSubsystem.simpleMotionMagicTest(leftTarget, rightTarget);
    
    SmartDashboard.putNumber("leftTarget",leftTarget);
    SmartDashboard.putNumber("RightTarget", rightTarget);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveSubsystem.feed();
    Robot.smartDashboardSubsystem.updateEncoderValue();
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
    System.out.println("Interrupted Forward Advance");
  }
}
