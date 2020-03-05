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
import frc.robot.subsystems.TalonDriveSubsystem;

public class DriveTurnCommand extends Command {
  private int leftTarget;
  private int rightTarget;
  private double turnDegrees;
  private double targetHeading;
  private int leftAddEncoder;
  private int rightAddEncoder; 
  

  /**
   * Turns right that many degrees
   * @param degrees How many degrees to turn right (clockwise)
   */
  public DriveTurnCommand(double degrees) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    requires(Robot.navXSubsystem);
    turnDegrees = degrees;
    leftAddEncoder = (int) Math.round(RobotMap.encoderUnitsPerRobotRotation * turnDegrees / 360);
    rightAddEncoder = (int) Math.round(RobotMap.encoderUnitsPerRobotRotation * turnDegrees / 360);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    TalonDriveSubsystem.drive.setSafetyEnabled(false);
    //NOTE: This is *not* configured to work with the NavX anymore: it is purely based on encoder tics
    //We could (and maybe should) rewrite it to use the NavX as an auxiliary input for more accuracy.

    leftTarget = Robot.driveSubsystem.getLeftEncoder() + leftAddEncoder;
    rightTarget = Robot.driveSubsystem.getRightEncoder() - rightAddEncoder;
    targetHeading = Robot.navXSubsystem.getYaw() + turnDegrees;
    SmartDashboard.putNumber("leftTarget",leftTarget);
    SmartDashboard.putNumber("delta encoder", leftAddEncoder);
    SmartDashboard.putNumber("RightTarget", rightTarget);
    SmartDashboard.putNumber("TargetHeading", targetHeading);
    Robot.driveSubsystem.simpleMotionMagic(leftTarget, rightTarget);
    System.out.println("Turning init done.");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.smartDashboardSubsystem.updateEncoderValue();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveSubsystem.isOnTarget(leftTarget,rightTarget,100, targetHeading);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Done turning.");
    TalonDriveSubsystem.drive.setSafetyEnabled(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
