/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Goal: Shuffleboard (show info as widgets and get driving camera feeds from Pi) (Jack and I)

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.*;

import frc.robot.commands.*;
import frc.robot.Robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * Shuffleboard (show info as widgets and get driving camera feeds from Pi) (Jack and Peter)
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

    public void setupShuffleboard(){

         ShuffleboardTab displays = Shuffleboard.getTab("Displays");
         Shuffleboard.selectTab("Displays");

        //Speed of Encoders
        speedometerLayout = displays.getLayout("Speedometers", BuiltInLayouts.kList).withSize(2,3).withPosition(0,0);
        leftSpeedEntry = displays.getLayout("Speedometers").add("Speed of Left Encoder", 40).withWidget(BuiltInWidgets.kDial).getEntry();
        rightSpeedEntry = displays.getLayout("Speedometers").add("Speed of Right Encoder", 60).withWidget(BuiltInWidgets.kDial).getEntry();
       
        //Voltage
        voltageEntry = 
        displays.add("Battery Voltage", 20).withPosition(3,0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 14)).getEntry();

        //Gyro
        displays.add("Gyro Yaw", Robot.navXSubsystem.getNavX()).withWidget(BuiltInWidgets.kGyro);

        //Turret Rotation
        turretEntry = displays.add("Turret Rotation", 10).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max",360)).getEntry();

        wallFollowerLayout = displays.getLayout("Wall Follower", BuiltInLayouts.kList).withSize(2,2).withPosition(4, 0);
        //Can we activate wall follower?  If so, shows Green Light
        // wallFollowerPossibleEntry = displays.getLayout("Wall Follower").add("Wall Follow Possible", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    }

    /**
     * Create buttons that trigger commands on SmartDashboard. 
     * Currently nonfunctional
     */
    public void setupCommandsForTesting(){
      ShuffleboardTab testCommands = Shuffleboard.getTab("Test Commands");
      Shuffleboard.selectTab("Test Commands");
      //Run every motor forwards and backwards and solenoids up and down

      //Intake Motors Test
      ShuffleboardLayout intakeCommands = testCommands
       .getLayout("Intake", BuiltInLayouts.kList)
       .withSize(2, 9)
       .withPosition(0, 0);
     //  .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        intakeCommands.add(new IntakeInCommand());
        intakeCommands.add(new IntakeReverseCommand());
        intakeCommands.add(new IntakeUpCommand());
        intakeCommands.add(new IntakeDownCommand());
        intakeCommands.add(new IntakeLoaderUpCommand());
        intakeCommands.add(new IntakeLoaderDownCommand());
        intakeCommands.add(new IntakeMagazineInCommand());
        intakeCommands.add(new IntakeMagazineOutCommand());   
        intakeCommands.add(new IntakeStandbyCommand());

      //Shooter Motors Test
      ShuffleboardLayout shooterCommands = testCommands
       .getLayout("Shooter", BuiltInLayouts.kList)
       .withSize(2, 4)
       .withPosition(2, 0);
  //  .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        shooterCommands.add(new ShootCommand());
        shooterCommands.add(new ShootManuallyCommand());
        shooterCommands.add(new ShootEndCommand());
        shooterCommands.add(new ShooterCenterOnVisionCrybabyCommand());
 
        //Drive Train Motors Test

        ShuffleboardLayout driveCommands = Shuffleboard.getTab("Test Commands")
        .getLayout("Drive", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(4, 0);
   //  .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
 
        driveCommands.add(new DriveManuallyCommand());
        driveCommands.add(new DriveStopCommand());
        driveCommands.add(new DriveFollowWallCommand());
        driveCommands.add(new DriveZeroEncodersCommand());
 
        //Climber Test

        ShuffleboardLayout climberCommands = Shuffleboard.getTab("Test Commands")
        .getLayout("Climber", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(6, 0);
   //  .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
 
        climberCommands.add(new ClimbExtendCommand());
        climberCommands.add(new ClimbRetractCommand());
        climberCommands.add(new ClimbWinchUpCommand());
        climberCommands.add(new ClimbWinchDownCommand());
        climberCommands.add(new ClimbEndClimbCommand());

    }

    public void updateShuffleboardEntries(){
        leftSpeedEntry.setDouble(Robot.driveSubsystem.getLeftEncoderSpeed());
        rightSpeedEntry.setDouble(Robot.driveSubsystem.getRightEncoderSpeed());
        voltageEntry.setDouble(RobotController.getBatteryVoltage());
        turretEntry.setDouble(240);
        //wallFollowerPossibleEntry.setBoolean(Robot.ultrasonicSubsystem.checkWallFollowerPossible());
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new ShuffleboardSetupCommand());
      }

}
