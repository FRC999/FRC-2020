/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * @see www.maxbotix.com/firstRobotics
 * sensor returns a value from 0-4095
 * factor to convert to inches: 0.125
 */
public class UltrasonicSensorSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final AnalogInput ultrasonicLeft = new AnalogInput(RobotMap.ultrasonicInputChannelLeft);
  private final AnalogInput ultrasonicRight = new AnalogInput(RobotMap.ultrasonicInputChannelRight);

  public double getDistanceInInches() {
    double retVal = ultrasonicLeft.getValue() * RobotMap.ultrasonicValueToInchesConversionFactor;
    return retVal;
  }

  public double getUltrasonicLeftDistanceInMM() {
    double retVal = ultrasonicLeft.getValue() * RobotMap.ultrasonicValueToMMConversionFactor;
    return retVal;
  }

  public double getUltrosonicRightDistanceInMM() {
    double retVal = ultrasonicRight.getValue() * RobotMap.ultrasonicValueToMMConversionFactor;
    return retVal;
  }

  public int getUltrasonicLeftDistanceInRaw() {
    int retVal = ultrasonicLeft.getValue();
    return retVal;
  }

  public int getUltrasonicRightDistanceInRaw() {
    int retVal = ultrasonicRight.getValue();
return retVal;
}

public boolean checkLeftWallFollowerPossible(){
  if (getUltrasonicLeftDistanceInMM() < 3500) {
    return true;
  }
  else
  {
    return false;
  }

  

}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
      
  }

//Wall Crawler

//Need to maintain a distance of 45.72 cm from the wall.
//

}
