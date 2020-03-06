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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;


/**
 * Add your docs here.
 */
public class SmartDashboardSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public SmartDashboardSubsystem() {
    SmartDashboard.putString("Friendly", "Good Morning!");
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

  public void updateShooterValues() {
    SmartDashboard.putString("Target Side", Robot.shooterSubsystem.whichSide());
    //SmartDashboard.putNumber("Tilt Encoder", Robot.shooterSubsystem.gettiltEncoder());
    SmartDashboard.putNumber("Pan Encoder", Robot.shooterSubsystem.getPanEncoder());
    SmartDashboard.putNumber("Tilt Pot", Robot.shooterSubsystem.getTiltPot());
  }

  public void updateEncoderValue() {
    SmartDashboard.putNumber("left encoder", Robot.driveSubsystem.getLeftEncoder());
    SmartDashboard.putNumber("right encoder", Robot.driveSubsystem.getRightEncoder());
  }

  public void updateNavXValues() {
    SmartDashboard.putNumber("NavX Pitch", Robot.navXSubsystem.getPitch());
    SmartDashboard.putNumber("Navx Roll", Robot.navXSubsystem.getRoll());
    SmartDashboard.putNumber("NavX Yaw", Robot.navXSubsystem.getYaw());
  }

  public void updateUltrasonicValues() {
    SmartDashboard.putNumber("ultrasonic 1 raw value", Robot.ultrasonicSubsystem.getUltrasonicLeftDistanceInRaw());
    SmartDashboard.putNumber("ultrasonic 1 mm value", Robot.ultrasonicSubsystem.getUltrasonicLeftDistanceInMM());
    SmartDashboard.putNumber("ultrasonic 2 raw value", Robot.ultrasonicSubsystem.getUltrasonicRightDistanceInRaw());
    SmartDashboard.putNumber("ultrasonic 2 mm value", Robot.ultrasonicSubsystem.getUltrosonicRightDistanceInMM());
  }
  public void updateControlPanelValues() {
    SmartDashboard.putNumber("control panel quad encoder raw value", Robot.controlPanelSubsystem.readEncoderRaw() );
    SmartDashboard.putNumber("control panel quad encoder in revolutions ", Robot.controlPanelSubsystem.readEncoderRevolutions());
 
    Robot.controlPanelSubsystem.updateColorState();
    SmartDashboard.putNumber("Spotted Color: Red", Robot.controlPanelSubsystem.getCurrentColor().red );
    SmartDashboard.putNumber("Spotted Color: Green", Robot.controlPanelSubsystem.getCurrentColor().green );
    SmartDashboard.putNumber("Spotted Color: Blue", Robot.controlPanelSubsystem.getCurrentColor().blue);
    SmartDashboard.putNumber("Spotted Distance: ", Robot.controlPanelSubsystem.getProximity());
    if (Robot.controlPanelSubsystem.getSuspectedColor() != null) {
      SmartDashboard.putString("SuspectedColor: ", Robot.controlPanelSubsystem.getSuspectedColor().toString());
    }
    SmartDashboard.putString("testColors",Robot.controlPanelSubsystem.getGameTargetColor().getName()); // "want "+Robot.controlPanelSubsystem.getGameTargetColor().getName() +" now " +  Robot.controlPanelSubsystem.getSuspectedColor(Robot.controlPanelSubsystem.getSeenColor()).getName()
 
  }

  public void stackTrace(String s)
  {
    if (s != null)
    SmartDashboard.putString("print stack trace", s);
  }

  public void updateMatchTimeAndBatteryVoltage() {
    SmartDashboard.putNumber("MATCH TIME LEFT (s)", DriverStation.getInstance().getMatchTime());
    SmartDashboard.putNumber("battery voltage", RobotController.getBatteryVoltage());
  }

  public void updateAllDisplays() {
    updateNavXValues();
    updateUltrasonicValues();
    updateControlPanelValues();
    updateMatchTimeAndBatteryVoltage();
    updateEncoderValue();
  }
}
