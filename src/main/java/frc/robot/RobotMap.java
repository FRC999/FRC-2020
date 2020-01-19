/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.

  // Drivetrain Motor Controllers
  public static int frontLeftDriveMotorController = 1;
  public static int backLeftDriveMotorController = 2;
  public static int frontRightDriveMotorController = 4;
  public static int backRightDriveMotorController = 3;

  public static int kShooterMotorID = 5;
  public static int panMotorID = 6;
  public static int tiltMotorID = 7;

  // Driver Input Devices
  public static int leftJoystickPort = 0;

   public static int intakeMotor1ID = 8;
   public static int intakeMotor2ID = 9;

  //Closed loop constants
  public final static int encoderUnitsPerShaftRotation = 4096;
  public final static int encoderUnitsPerRobotRotation = 51711;

   // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
