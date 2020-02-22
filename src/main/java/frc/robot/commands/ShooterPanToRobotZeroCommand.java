/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ShooterPanToRobotZeroCommand extends Command {
  public ShooterPanToRobotZeroCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.shooterSubsystem);
  }

  boolean readyAtStart = false;
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (isFinished())
    readyAtStart = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!readyAtStart)
    Robot.shooterSubsystem.pan(0.5 * Robot.shooterSubsystem.getWhichWayToTurnToGetToZero());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean retVal = false;
    if ((Robot.shooterSubsystem.getPanEncoder() <= 20) || (Robot.shooterSubsystem.getPanEncoder() >= (RobotMap.shooterPanMotorEncoderTicksPerTurretRotation -20)) ) {
retVal = true;
    }

    return retVal;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.shooterSubsystem.pan(0);
    readyAtStart = false;// probably don't need this
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.shooterSubsystem.pan(0);
  }
}
