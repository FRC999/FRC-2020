/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ControlPanelMoveToTargetCommand extends Command {
  double targetRev;
  double targetTicks;
  /** @param target the number of revolutions, positive or negative, to turn the control panel */
  public ControlPanelMoveToTargetCommand(double target) {
    targetRev = target;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.controlPanelSubsystem);
    targetTicks = Robot.controlPanelSubsystem.controlPanelTargetRevolutionsToQuadEncoderTicks(target);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
Robot.controlPanelSubsystem.zeroEncoder();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    
      Robot.controlPanelSubsystem.moveTalonInDirection(targetTicks,0.5);


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
   boolean retVal = false;
    if (( (Math.signum(targetTicks) == 1) && (Robot.controlPanelSubsystem.readEncoderRaw() <= targetTicks)) || ((Math.signum(targetTicks) == -1) && (Robot.controlPanelSubsystem.readEncoderRaw() >= targetTicks)))
     {retVal = false;}
      else {retVal = true;}
   
    return retVal;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.controlPanelSubsystem.stopTalon();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
