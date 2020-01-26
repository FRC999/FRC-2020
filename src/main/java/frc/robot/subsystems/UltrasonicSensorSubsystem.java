/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.RobotMap;

/**
 * @see www.maxbotix.com/firstRobotics
 * sensor returns a value from 0-4095
 * factor to convert to inches: 0.125
 */
public class UltrasonicSensorSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final AnalogInput ultrasonic1 = new AnalogInput(RobotMap.ultrasonicInputChannel1);
  private final AnalogInput ultrasonic2 = new AnalogInput(RobotMap.ultrasonicInputChannel2);
  //channel on the roborio section DIO, to trigger a reading from the ultrasonic sensor
  private final DigitalOutput ultrasonicTrigger1 = new DigitalOutput(RobotMap.ultrasonicTriggerChannel1);
  private final DigitalOutput ultrasonicTrigger2 = new DigitalOutput(RobotMap.ultrasonicTriggerChannel2);

  /**sends a pulse to trigger sensor 1 to send in a new value. */
  public void pulseSensor1() {
      ultrasonicTrigger1.pulse(RobotMap.ultrasonicTriggerTime);
  }
    /**sends a pulse to trigger sensor 2 to send in a new value. */
    public void pulseSensor2() {
      ultrasonicTrigger2.pulse(RobotMap.ultrasonicTriggerTime);
  }

  public double getDistanceInInches() {
    double retVal = ultrasonic1.getValue() * RobotMap.ultrasonicValueToInchesConversionFactor;
    return retVal;
  }
  public double getSensor1DistanceInMM() {
    double retVal = ultrasonic1.getValue() * RobotMap.ultrasonicValueToMMConversionFactor;
    return retVal;
  }

public double getSensor2DistanceInMM() {
  double retVal = ultrasonic2.getValue() * RobotMap.ultrasonicValueToMMConversionFactor;
  return retVal;
}


public double getSensor1DistanceInRaw() {
  double retVal = ultrasonic1.getValue();
  return retVal;
}

public double getSensor2DistanceInRaw() {
double retVal = ultrasonic2.getValue();
return retVal;
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
      
  }

  /*private static class UltrasonicChecker extends Thread {
@Override

  } */
}
