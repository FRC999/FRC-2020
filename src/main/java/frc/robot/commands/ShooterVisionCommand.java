/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ShooterVisionCommand extends Command {
  public ShooterVisionCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.shooterSubsystem);
  }
  public String loc = "";
  public int counter = 0;
  public int counterNum = 5;
  public boolean bounds = false;
  public String side = "";

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    counter = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.shooterSubsystem.centerShooter();
    side = Robot.shooterSubsystem.whichSide();
    if (side == ("Center")) {
      counter +=1;
    } else {
      counter = 0;
    }
    //bounds = Robot.shooterSubsystem.inBounds(Robot.shooterSubsystem.getX());
    //side = Robot.shooterSubsystem.whichSide();
/*
    switch (side) {
      case "Left" : {
        Robot.shooterSubsystem.panMotorController.set(RobotMap.shooterPanSpeed);
        System.out.println("TARGET LEFT OF CENTER");
        loc = "Left";
        counter = 0;
      }
      break;
      case "Center" : {
        Robot.shooterSubsystem.panMotorController.set(0);
        System.out.println("TARGET IN CENTER");
        loc = "Center";
        counter+=1;
      }
      break;
      case "Right" : {
        Robot.shooterSubsystem.panMotorController.set((RobotMap.shooterPanSpeed)*-1);
        System.out.println("TARGET RIGHT OF CENTER");
        loc = "Right";
        counter = 0;
      }
      break;
      case "Out Of Bounds" : {
        Robot.shooterSubsystem.panMotorController.set(0);
        System.out.println("TARGET OUT OF BOUNDS");
        loc = "Out Of Bounds";
        counter = 0;
      }
      default : {
        Robot.shooterSubsystem.panMotorController.set(0);
        System.out.println("DEFAULT");
        loc = "Default";
        counter = 0;
      }
    }
    if (Robot.shooterSubsystem.panMotorController.get() == 0) {
    }
    */
  }
  
  // Make this return true when this Command no longer needs to run execute()
  

  @Override
  protected boolean isFinished() {
    boolean state = false;
    if (counter >= 3) {
      System.out.println("FINISHED TRACKING");
      state = true;
    }
    return state;
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

