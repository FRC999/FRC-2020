/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

/**
 * OPERATOR INPUT This class is the glue that binds the controls on the physical
 * operator interface to the commands and command groups that allow control of
 * the robot.
 */
public class OI {
  public Joystick leftJoystick = new Joystick(RobotMap.leftJoystickPort);
  Button stopButton = new JoystickButton(leftJoystick, 1);
  Button showEncoderButton = new JoystickButton(leftJoystick, 2);
  Button zeroEncoderButton = new JoystickButton(leftJoystick, 3);
  Button zeroYawButton = new JoystickButton(leftJoystick, 4);
  Button climberExtendButton = new JoystickButton(leftJoystick, 5);
  Button climberClimbButton = new JoystickButton(leftJoystick, 6);
  Button testMotionMagicButton = new JoystickButton(leftJoystick , 7);

  public OI() { // Setup All Commands Here
    zeroEncoderButton.whenPressed(new ZeroDriveEncodersCommand());
    zeroYawButton.whenPressed(new ZeroYawCommand());
    testMotionMagicButton.whenPressed(new AutoMotionMagicCommand());
    
  }
}
