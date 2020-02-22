/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**as the robot turns, keeps the shooter at the same position relative to the field rather than to the robot.
 * Steps: get the starting angle theta bot-to-field and the starting angle theta shooter-to-bot.
 * From this, fix an ideal theta shooter to field
 * every iteration:
 * get the current theta btf and theta stb  stf = stb + btf  stf - btf = stb
 * using the equation stf- btf = stb, get the ideal angle the shooter should be at and rotate to it.
 * when that angle is reached, stop the shooter from moving.
 * 
 * 
*/
public class ShooterStayCommand extends Command {
  public ShooterStayCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.shooterSubsystem);
    requires(Robot.navXSubsystem);
    
  }

  double initialThetaBotToField;
  double currentThetaBotToField;
  double initialThetaShooterToBot;
  double currentThetaShooterToBot;
  double idealThetaShooterToBot;
  double idealThetaShooterToField;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initialThetaBotToField = Robot.navXSubsystem.getYaw();
    initialThetaShooterToBot = Robot.shooterSubsystem.getHeadingDegreesFromPanEncoderAdjustValue();
    idealThetaShooterToField = initialThetaBotToField + initialThetaShooterToBot;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentThetaBotToField = Robot.navXSubsystem.getYaw();//TODO: make sure these scales sync
    currentThetaShooterToBot = Robot.shooterSubsystem.getHeadingDegreesFromPanEncoderAdjustValue();
    idealThetaShooterToBot = idealThetaShooterToField - currentThetaBotToField;
    if ((currentThetaShooterToBot < idealThetaShooterToBot + 10) && (currentThetaShooterToBot > idealThetaShooterToBot - 10))
    {Robot.shooterSubsystem.pan(0);}
    else{Robot.shooterSubsystem.pan(Robot.shooterSubsystem.getWhichWayToTurnToGetToAdjustedEncoderValue(Robot.shooterSubsystem.getPanEncoderAdjustValueFromHeading(idealThetaShooterToBot)) * 0.5);}
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.shooterSubsystem.pan(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.shooterSubsystem.pan(0);
  }
}
