package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.commands.*;
import frc.robot.Robot;

public class ShuffleboardSubsystem extends Subsystem{

    NetworkTableEntry leftSpeedEntry;
    NetworkTableEntry rightSpeedEntry;
    ShuffleboardLayout speedometerLayout;
    
    NetworkTableEntry voltageEntry;

    NetworkTableEntry turretEntry;

    public ShuffleboardSubsystem(){
    }

    public void setupShuffleboard(){
        ShuffleboardTab displays = Shuffleboard.getTab("Displays");
        Shuffleboard.selectTab("Displays");


        //Speed of Encoders
        speedometerLayout = Shuffleboard.getTab("Displays").getLayout("Speedometers", BuiltInLayouts.kList).withSize(2,3).withPosition(0,0);
        leftSpeedEntry = 
        Shuffleboard.getTab("Displays").getLayout("Speedometers").add("Speed of Left Encoder", 40).withWidget(BuiltInWidgets.kDial).getEntry();
        rightSpeedEntry = 
        Shuffleboard.getTab("Displays").getLayout("Speedometers").add("Speed of Right Encoder", 60).withWidget(BuiltInWidgets.kDial).getEntry();
       
        //Voltage
        voltageEntry = 
        Shuffleboard.getTab("Displays").add("Battery Voltage", 20).withPosition(3,0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 14)).getEntry();

        //Gyro
        Shuffleboard.getTab("Displays").add("Gyro Yaw", Robot.navXSubsystem.getNavX()).withWidget(BuiltInWidgets.kGyro);

        //Turret Rotation
        turretEntry = Shuffleboard.getTab("Displays").add("Turret Rotation", 10).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max",360)).getEntry();

       //Test Entry
        Shuffleboard.getTab("Displays").add("Test", 3.14);
    }

    public void updateShuffleboardEntries(){
        leftSpeedEntry.setDouble(Robot.driveSubsystem.getLeftEncoderSpeed());
        rightSpeedEntry.setDouble(Robot.driveSubsystem.getRightEncoderSpeed());
        voltageEntry.setDouble(RobotController.getBatteryVoltage());
        turretEntry.setDouble(240);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new ShuffleboardSetupCommand());
      }

}

//Needs: Gyro values, Camera, Encoder Positions, Heading direction, Speedometer, Battery Voltage 