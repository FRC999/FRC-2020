/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardCommand extends Command {
  private static final int driveDistance = 14000;
  DriveSubsystem driveSubsystem = Robot.driveSubsystem;
  int startPoint;
  public DriveForwardCommand() {
    requires(Robot.driveSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    startPoint = 0;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //This is to allow me to use the same command for the first and second leg
    // This is only called before the command is run the first time!!!
    startPoint = driveSubsystem.getLeftEncoder();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveSubsystem.ManualDrive(.5, 0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(driveSubsystem.getLeftEncoder() >= driveDistance+startPoint){
      return true;//exit method and command
    }
    return false; //keep going
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
