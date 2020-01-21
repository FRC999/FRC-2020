/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotMap;

/**
 * @see www.maxbotix.com/firstrobotics
 * sensor returns a value from 0-4095
 * factor to convert to inches: 0.125
 */
public class UltrasonicSensorSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final AnalogInput ultrasonic = new AnalogInput(RobotMap.ultrasonicInputChannel);

  public double getDistanceInInches() {
    double retVal = ultrasonic.getValue() * RobotMap.ultrasonicValueToInchesConversionFactor;
    return retVal;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
