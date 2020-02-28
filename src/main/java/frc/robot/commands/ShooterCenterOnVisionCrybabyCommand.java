/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCenterOnVisionCrybabyCommand extends Command {
  public ShooterCenterOnVisionCrybabyCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.shooterSubsystem);
  }
  public String loc = "";
  public int counter = 0;
  public int counterNum = 5;
  public boolean bounds = false;
  public String side = "";
  public double pos = 0;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    counter = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    //bounds = Robot.shooterSubsystem.inBounds(Robot.shooterSubsystem.getX());
    //side = Robot.shooterSubsystem.whichSide(Robot.shooterSubsystem.getX());

    switch (side) {
      case "Left" : {
        pos = ShooterSubsystem.panMotorController.getSelectedSensorPosition()+3;
        ShooterSubsystem.panMotorController.set(ControlMode.Position, pos);
        System.out.println("TARGET LEFT OF CENTER");
        loc = "Left";
        counter = 0;
      }
      break;
      case "Center" : {
        ShooterSubsystem.panMotorController.set(0);
        System.out.println("TARGET IN CENTER");
        loc = "Center";
        counter+=1;
      }
      break;
      case "Right" : {
        pos = ShooterSubsystem.panMotorController.getSelectedSensorPosition()-3;
        ShooterSubsystem.panMotorController.set(ControlMode.Position, pos);

        System.out.println("TARGET RIGHT OF CENTER");
        loc = "Right";
        counter = 0;
      }
      break;
      case "Out Of Bounds" : {
        ShooterSubsystem.panMotorController.set(0);
        System.out.println("TARGET OUT OF BOUNDS");
        loc = "Out Of Bounds";
        counter = 0;
      }
      default : {
        ShooterSubsystem.panMotorController.set(0);
        System.out.println("DEFAULT");
        loc = "Default";
        counter = 0;
      }
    }
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean state = false;
    if(loc == "Center" && counter >= counterNum) {
      state = true;
    }
    System.out.println("FINISHED TRACKING");
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
