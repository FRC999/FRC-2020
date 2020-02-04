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

public class TurnRightX extends Command {
  private int leftTarget;
  private int rightTarget;
  private double turnDegrees;
  
  public TurnRightX(double degrees) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    turnDegrees = degrees;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSubsystem.driveTrainBrakeMode();
    //NOTE: This is *not* configured to work with the NavX anymore: it is purely based on encoder tics
    //We could (and maybe should) rewrite it to use the NavX as an auxiliary input for more accuracy.
    leftTarget = Robot.driveSubsystem.getLeftEncoder() + (int) Math.round(RobotMap.encoderUnitsPerRobotRotation * turnDegrees / 360);
    rightTarget = Robot.driveSubsystem.getRightEncoder() - (int) Math.round(RobotMap.encoderUnitsPerRobotRotation * turnDegrees / 360);
    Robot.driveSubsystem.simpleMotionMagicTest(leftTarget, rightTarget);
    System.out.println("Turning init done.");
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
    return Robot.driveSubsystem.isOnTarget(leftTarget,rightTarget,100);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Done turning.");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
