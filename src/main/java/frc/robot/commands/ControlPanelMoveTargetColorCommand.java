/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ControlPanelSubsystem;

public class ControlPanelMoveTargetColorCommand extends Command {
  
  ControlPanelSubsystem.PanelColors colorUnderSensor;
  ControlPanelSubsystem.PanelColors colorWantedUnderSensor;
  double encoderTarget;
  public ControlPanelMoveTargetColorCommand() {
    // Use requires() here to declare subsystem dependencie
    // eg. requires(chassis);
    requires(Robot.controlPanelSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.controlPanelSubsystem.updateColorState();
    colorWantedUnderSensor = Robot.controlPanelSubsystem.getGameTargetColor();
    colorUnderSensor = Robot.controlPanelSubsystem.getSuspectedColor(Robot.controlPanelSubsystem.getSeenColor());
    encoderTarget = Robot.controlPanelSubsystem.getPathToDesiredColor(colorUnderSensor, colorWantedUnderSensor);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.controlPanelSubsystem.moveTalonInDirection(encoderTarget);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
