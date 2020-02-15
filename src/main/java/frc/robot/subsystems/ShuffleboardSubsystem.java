package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.commands.*;
import frc.robot.Robot;

public class ShuffleboardSubsystem extends Subsystem{

    public ShuffleboardSubsystem(){
        ShuffleboardTab displays = Shuffleboard.getTab("Displays");
        Shuffleboard.selectTab("Displays");
    }

    public void setup(){
        Shuffleboard.getTab("Displays").add("Test", 3.14);
        Shuffleboard.getTab("Displays").add("Gyro", Robot.navXSubsystem.getNavX()).withWidget(BuiltInWidgets.kGyro);
        Shuffleboard.getTab("Displays").add("Speed", Robot.driveSubsystem.getLeftEncoderSpeed()).withWidget(BuiltInWidgets.kDial);
    }



    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new ShuffleboardSetupCommand());
      }

}

//Needs: Gyro values, Camera, Encoder Positions, Heading direction, Speedometer, Battery Voltage 