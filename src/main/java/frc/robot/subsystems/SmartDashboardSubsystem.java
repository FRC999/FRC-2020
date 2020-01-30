/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.UpdateAllSmartDashboard;

/**
 * Add your docs here.
 */
public class SmartDashboardSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public SmartDashboardSubsystem(){
    SmartDashboard.putString("Friendly", "Good Morning!");
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new UpdateAllSmartDashboard());
  }
  public void updateNavXValues(){
    SmartDashboard.putNumber("NavX Pitch", Robot.navXSubsystem.getPitch());
    SmartDashboard.putNumber("Navx Roll", Robot.navXSubsystem.getRoll());
    SmartDashboard.putNumber("NavX Yaw", Robot.navXSubsystem.getYaw());
  }
  public void updateAllDisplays(){
    updateNavXValues();
  }
}
