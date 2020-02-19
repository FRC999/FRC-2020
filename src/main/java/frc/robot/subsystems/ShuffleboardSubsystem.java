/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Goal: Shuffleboard (show info as widgets and get driving camera feeds from Pi) (Jack and I)

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.*;

import frc.robot.commands.*;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.commands.ShuffleboardSetupCommand;
import frc.robot.commands.SmartDashboardUpdateAllCommand;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;

//Shuffleboard (show info as widgets and get driving camera feeds from Pi) (Jack and I)

/**
 * Add your docs here.
 */
public class ShuffleboardSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  int test; 

  NetworkTableEntry voltageEntry;

  NetworkTableEntry turretEntry;

  ShuffleboardLayout speedometerLayout;
  NetworkTableEntry leftSpeedEntry;
  NetworkTableEntry rightSpeedEntry;

  ShuffleboardLayout wallFollowerLayout;
  NetworkTableEntry wallFollowerPossibleEntry;
  
// Constructor
  public ShuffleboardSubsystem() {
    test=5;
  }

/*

  public void initializeShuffleboard(){
    //Create Tabs
    ShuffleboardTab dataValuesTab = Shuffleboard.getTab("Data Values");
    ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");


    //Send Simple Data to Tabs
    //Data Tab
    Shuffleboard.getTab("Data Values").add("Test", Robot.shuffleBoardSubsystem.test);
  }
  
  */

    public void setupShuffleboard(){

         ShuffleboardTab displays = Shuffleboard.getTab("Displays");
         Shuffleboard.selectTab("Displays");

        //Speed of Encoders
        speedometerLayout = Shuffleboard.getTab("Displays").getLayout("Speedometers", BuiltInLayouts.kList).withSize(2,3).withPosition(0,0);
        leftSpeedEntry = Shuffleboard.getTab("Displays").getLayout("Speedometers").add("Speed of Left Encoder", 40).withWidget(BuiltInWidgets.kDial).getEntry();
        rightSpeedEntry = Shuffleboard.getTab("Displays").getLayout("Speedometers").add("Speed of Right Encoder", 60).withWidget(BuiltInWidgets.kDial).getEntry();
       
        //Voltage
        voltageEntry = 
        Shuffleboard.getTab("Displays").add("Battery Voltage", 20).withPosition(3,0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 14)).getEntry();

        //Gyro
        Shuffleboard.getTab("Displays").add("Gyro Yaw", Robot.navXSubsystem.getNavX()).withWidget(BuiltInWidgets.kGyro);

        //Turret Rotation
        turretEntry = Shuffleboard.getTab("Displays").add("Turret Rotation", 10).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max",360)).getEntry();

        wallFollowerLayout = Shuffleboard.getTab("Displays").getLayout("Wall Follower", BuiltInLayouts.kList).withSize(2,2).withPosition(4, 0);
        //Can we activate wall follower?  If so, shows Green Light
        wallFollowerPossibleEntry = Shuffleboard.getTab("Displays").getLayout("Wall Follower").add("Wall Follow Possible", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
       // Shuffleboard.getTab("Displays").getLayout("Wall Follower").add("Wall Follow", new MaintainDistanceCommand());

       //Test Entry
        Shuffleboard.getTab("Displays").add("Test", 3.14);
        
    }

    public void updateShuffleboardEntries(){
        leftSpeedEntry.setDouble(Robot.driveSubsystem.getLeftEncoderSpeed());
        rightSpeedEntry.setDouble(Robot.driveSubsystem.getRightEncoderSpeed());
        voltageEntry.setDouble(RobotController.getBatteryVoltage());
        turretEntry.setDouble(240);
        wallFollowerPossibleEntry.setBoolean(Robot.ultrasonicSubsystem.checkWallFollowerPossible());
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new ShuffleboardSetupCommand());
      }

}
