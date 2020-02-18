/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Goal: Shuffleboard (show info as widgets and get driving camera feeds from Pi) (Jack and I)

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.commands.InitializeShuffleboardCommand;
import frc.robot.commands.SmartDashboardUpdateAllCommand;

//Shuffleboard (show info as widgets and get driving camera feeds from Pi) (Jack and I)

/**
 * Add your docs here.
 */
public class ShuffleBoardSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  int test; 
// Constructor
  public ShuffleBoardSubsystem() {
    test=5;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new InitializeShuffleboardCommand());
  }

  public void initializeShuffleboard(){
    //Create Tabs
    ShuffleboardTab dataValuesTab = Shuffleboard.getTab("Data Values");
    ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");

    //Send Simple Data to Tabs
    //Data Tab
    Shuffleboard.getTab("Data Values").add("Test", Robot.shuffl8eBoardSubsystem.test);
    
  }



  public void updateEncoderValue() {
    Shuffleboard.getTab("Data Values").add("Left Encoder", Robot.driveSubsystem.getLeftEncoder());
    Shuffleboard.getTab("Data Values").add("Right Encoder", Robot.driveSubsystem.getRightEncoder());
  }

  public void updateNavXValues() {
    Shuffleboard.getTab("Data Values").add("NavX Pitch", Robot.navXSubsystem.getPitch());
    Shuffleboard.getTab("Data Values").add("NavX Roll", Robot.navXSubsystem.getRoll());
    Shuffleboard.getTab("Data Values").add("NavX Yaw", Robot.navXSubsystem.getYaw());
  }

  public void updateUltrasonicValues() {
    Shuffleboard.getTab("Data Values")
    .add("Ultrasonic 1 Raw Value", Robot.ultrasonicSubsystem.getSensor1DistanceInRaw());
    Shuffleboard.getTab("Data Values")
    .add("Ultrasonic 1 mm Value", Robot.ultrasonicSubsystem.getSensor1DistanceInMM());  
    Shuffleboard.getTab("Data Values")
    .add("Ultrasonic 2 Raw Value", Robot.ultrasonicSubsystem.getSensor2DistanceInRaw());
    Shuffleboard.getTab("Data Values")
    .add("Ultrasonic 2 mm Value", Robot.ultrasonicSubsystem.getSensor2DistanceInMM());
  }

  public void updateControlPanelValues() {
    // SmartDashboard.putNumber("control panel quad encoder raw value",
    // Robot.controlPanelSubsystem.readEncoderRaw() );
    // SmartDashboard.putNumber("control panel quad encoder in revolutions ",
    // Robot.controlPanelSubsystem.readEncoderRevolutions());
  }

  public void updateAllDisplays() {
    updateNavXValues();
    updateUltrasonicValues();
    updateControlPanelValues();
  }
}
