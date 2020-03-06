/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ShooterTestFangsCommand extends Command{
  public int c = 0;
  public ShooterTestFangsCommand(){
  }
  @Override
  protected void initialize(){
    c = 0;
  }

  @Override
  protected void execute(){
      Robot.shooterSubsystem.testTiltFangs();
      c++;
  }


  @Override
  protected boolean isFinished(){
    boolean t = false;  
    if (c >= 1) {
        t = true;
      }
      return t;
  }

  @Override
  protected void end() {
    Robot.shooterSubsystem.tiltMotorController.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}