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

public class AutoMotionMagicCommand extends Command {
  private static final int testEncoderVal = 50000;
  public AutoMotionMagicCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    //Robot.driveSubsystem.configureDriveTrainControllersForSimpleMagic();

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    DriveSubsystem.drive.setSafetyEnabled(false);  // This should prevent the watchdog from complaining during movement using motionmagic
    Robot.driveSubsystem.simpleMotionMagicTest(testEncoderVal,  testEncoderVal);    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.smartDashboardSubsystem.updateEncoderValue();
    //DriveSubsystem.drive.feed();//It took us four hours to figure out we needed this line
    //"Watchdog does not behave nicely when it gets pissed off"
    // --- fixed by setting setSafetyEnabled to false durring this command -----
  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean retVal= false;
    if((Robot.driveSubsystem.getLeftEncoder() >= testEncoderVal) ||(Robot.driveSubsystem.getRightEncoder() >= testEncoderVal) ){
      retVal = true;//exit method and command
    }
    else
    retVal = false;//keep going
    
    return retVal;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    DriveSubsystem.drive.setSafetyEnabled(true);  // Reactivate watchdog after motionmagic
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
