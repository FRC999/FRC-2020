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

public class TurnAroundCommand extends Command {
  DriveSubsystem driveSubsystem = Robot.driveSubsystem;
  static final double targetFacing = 170; // cannot be 180, as the system wraps around automatically to -180 after it hits 180
  double testYaw = 0;
  public TurnAroundCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveSubsystem.manualDrive(0, .5);
    Robot.smartDashboardSubsystem.updateNavXValues();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
   boolean testVal= false;
    if(((Robot.navXSubsystem.getYaw() >= targetFacing)||(Robot.navXSubsystem.getYaw() <= -targetFacing)) && (testYaw > Robot.navXSubsystem.getYaw()) ){
       /* three conditions: 1 and 2, in the range around +-180; 3,
       // that it just changed over from positive to negative. 
       //We are assuming it goes clockwise; the NavX wraps from 180 degrees to -180.
    */
       testVal = true;
      testYaw = 0;
    }
    else 
    { testYaw = Robot.navXSubsystem.getYaw();
      testVal = false;}

    return testVal;
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
