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
//    Robot.driveSubsystem.SimpleMotionMagicTest(9, 9);
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    Robot.driveSubsystem.SimpleMotionMagicTest(testEncoderVal,  testEncoderVal);
    Robot.smartDashboardSubsystem.updateEncoderValue();
    DriveSubsystem.drive.feed();//It took us four hours to figure out we needed this line
    //"Watchdog does not behave nicely when it gets pissed off"
  
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
