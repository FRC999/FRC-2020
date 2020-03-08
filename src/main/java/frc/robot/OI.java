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
  public Joystick rightJoystick = new Joystick(RobotMap.rightJoystickPort);
  public Joystick buttonBox = new Joystick(RobotMap.buttonBoxPort);

  //RightJoystick
  Button runIntake = new JoystickButton(rightJoystick, 1);
  Button intakeEject = new JoystickButton(rightJoystick, 3);
  //Left Joystick

  //Button Board
  Button climberSolenoidForward = new JoystickButton(buttonBox, 1);
  Button winchMotor = new JoystickButton(buttonBox, 2);
  Button climberSolenoidReverse = new JoystickButton(buttonBox, 3);
  Button magazineInward = new JoystickButton(buttonBox, 4);
  Button loaderUp = new JoystickButton(buttonBox, 5);
  public Button shooterMotor = new JoystickButton(buttonBox, 6);
  Button magazineOutward = new JoystickButton(buttonBox, 7);
  Button loaderDown = new JoystickButton(buttonBox, 8);
  Button fangsFullyBack = new JoystickButton(buttonBox, 9);
  Button visionTracking = new JoystickButton(buttonBox, 10);
  Button fullShooter = new JoystickButton(buttonBox, 11); //loader + magazine + shooter
  Button zeroTurret = new JoystickButton(buttonBox, 12);
  
  public OI() {
     // Setup All Commands Here
     
     //right Joystick
     runIntake.whileHeld(new IntakeInCommand());
     runIntake.whenReleased(new IntakeUpCommand());
     intakeEject.whileHeld(new IntakeEject());
     //Left Joystick



    //Button Board
    climberSolenoidForward.whenPressed(new ClimbExtendCommand());
    winchMotor.whileHeld(new ClimbWinchUpCommand());
    climberSolenoidReverse.whenPressed(new ClimbRetractCommand());
    magazineInward.whileHeld(new IntakeMagazineInCommand());
    loaderUp.whileHeld(new IntakeLoaderUpCommand());
    shooterMotor.whileHeld(new ShooterRunWheelCommand());
    magazineOutward.whileHeld(new IntakeMagazineInCommand());
    loaderDown.whileHeld(new IntakeLoaderDownCommand());
    fangsFullyBack.whenPressed(new ShooterTiltGoToSetpointCommand());
    visionTracking.whileHeld(new ShooterVisionCommand());
    fullShooter.whileHeld(new ShooterFullCommand());
    zeroTurret.whileHeld(new ShooterTurretCenterCommand());
    
  }
}