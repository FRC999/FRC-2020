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
import frc.robot.subsystems.UltrasonicSensorSubsystem;

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

  public void updateEncoderValue() {
SmartDashboard.putNumber("left encoder",Robot.driveSubsystem.getLeftEncoder());
  }

  public void updateNavXValues(){
    SmartDashboard.putNumber("NavX Pitch", Robot.navXSubsystem.getPitch());
    SmartDashboard.putNumber("Navx Roll", Robot.navXSubsystem.getRoll());
    SmartDashboard.putNumber("NavX Yaw", Robot.navXSubsystem.getYaw());
  }

  public void updateUltrasonicValues() {
    SmartDashboard.putNumber("ultrasonic 1 raw value",Robot.ultrasonicSubsystem.getSensor1DistanceInRaw());
    SmartDashboard.putNumber("ultrasonic 1 mm value",Robot.ultrasonicSubsystem.getSensor1DistanceInMM());
    SmartDashboard.putNumber("ultrasonic 2 raw value",Robot.ultrasonicSubsystem.getSensor2DistanceInRaw());
    SmartDashboard.putNumber("ultrasonic 2 mm value",Robot.ultrasonicSubsystem.getSensor2DistanceInMM());
  }

  public void updateAllDisplays(){
    updateNavXValues();
    updateUltrasonicValues();
  }
}
