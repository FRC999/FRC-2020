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
  Button stopButton = new JoystickButton(leftJoystick, 1);
 // Button showAllButton = new JoystickButton(leftJoystick, 2);// not used
  Button zeroEncoderButton = new JoystickButton(leftJoystick, 3);
  Button zeroYawButton = new JoystickButton(leftJoystick, 4);
  Button turnAbsoluteTestButton = new JoystickButton(leftJoystick, 5);
//  Button climberClimbButton = new JoystickButton(leftJoystick, 6); // not used
  Button testMotionMagicButton = new JoystickButton(leftJoystick , 7);
  Button spin = new JoystickButton(leftJoystick, 8);
 /*
  Button zeroControlPanelEncoderButton = new JoystickButton(leftJoystick, 9);
  Button setControlPanelPositionButton = new JoystickButton(leftJoystick, 10);
  Button moveControlPanelAlongColorPathButton = new JoystickButton(leftJoystick, 11);
 */ // TODO: Put these back in when we have a control panel.
  // Button shooterDeployTiltFangsButton = new JoystickButton(leftJoystick, 12);
  Button bigManAuto = new JoystickButton(leftJoystick, 12);

  Button intakeExtendButton = new JoystickButton(rightJoystick, 1);

  

  Button climbExtendButton = new JoystickButton(buttonBox, 1);
  Button climbWinchUpButton = new JoystickButton(buttonBox, 2);
  Button climbRetractButton = new JoystickButton(buttonBox, 3);
  Button intakeMagazineInButton = new JoystickButton(buttonBox, 4);
  Button intakeLoaderUpButton = new JoystickButton(buttonBox, 5);
  Button shooterWheelControlButton = new JoystickButton(buttonBox, 6);
  Button intakeMagazineOutButton = new JoystickButton(buttonBox, 7);
  Button intakeLoaderDownButton = new JoystickButton(buttonBox, 8);
    Button shooterTiltFloorButton = new JoystickButton(buttonBox,9);
  Button shooterCryBabyButton = new JoystickButton(buttonBox, 10);
  Button shootAndLoadButton = new JoystickButton(buttonBox, 11);
  //TODO: this command needs to be tested
  Button shootTurretHomeButton = new JoystickButton(buttonBox, 12);

    //Button visionTestButton = new JoystickButton(buttonBox, 4);
  //Button shooterManualControlButton = new JoystickButton(buttonBox, 5);
 
  Button shooterTiltButton = new JoystickButton(rightJoystick, 11); // most recent tilt 3/5/2020
  
  //Button ShooterTiltGoToSetpointButton = new JoystickButton(buttonBox, 6);
  Button shooterFangsTestButton = new JoystickButton(rightJoystick, 9);
 // Button autoDriveForwardButton = new JoystickButton(buttonBox, 8);
  //Button autoTurnButton = new JoystickButton(buttonBox, 9);
  


  public OI() { // Setup All Commands Here
    zeroEncoderButton.whenPressed(new DriveZeroEncodersCommand());
    zeroYawButton.whenPressed(new NavXZeroYawCommand());
    testMotionMagicButton.whenPressed(new DriveForwardCommand(50000));
    spin.whenPressed(new DriveTurnCommand(90));
    stopButton.whileActive(new DriveStopCommand());
    turnAbsoluteTestButton.whenActive(new DriveTurnAbsoluteCommand(90));
 /*
    zeroControlPanelEncoderButton.whenPressed(new ControlPanelZeroEncoderCommand());
    setControlPanelPositionButton.whenPressed(new ControlPanelMoveToTargetCommand(3.5));
    moveControlPanelAlongColorPathButton.whenPressed(new ControlPanelMoveTargetColorCommand());
    */
    //shooterManualControlButton.whenPressed(new ShootManuallyCommand());
    bigManAuto.whenPressed(new ShootAndRunAuto(228.59, 1, true));

  

    climbExtendButton.whenPressed(new ClimbExtendCommand());
    climbWinchUpButton.whenPressed(new ClimbWinchUpCommand());
    climbRetractButton.whenPressed(new ClimbRetractCommand());

    intakeMagazineInButton.whileHeld(new IntakeMagazineInCommand());
  //  intakeMagazineInButton.whenReleased(new IntakeStandbyCommand());
    intakeMagazineOutButton.whileHeld(new IntakeMagazineOutCommand());
   // intakeMagazineOutButton.whenReleased(new IntakeStandbyCommand());
    intakeLoaderUpButton.whileHeld(new IntakeLoaderUpCommand());
    //intakeLoaderUpButton.whenReleased(new IntakeStandbyCommand());
    intakeLoaderDownButton.whileHeld(new IntakeLoaderDownCommand());
    //intakeLoaderDownButton.whenReleased(new IntakeStandbyCommand());

    intakeExtendButton.whenPressed(new IntakeDownCommand());
    intakeExtendButton.whenReleased( new IntakeUpCommand());

    shooterWheelControlButton.whenPressed(new ShooterWheelCommand());
    shooterCryBabyButton.whileHeld(new ShooterVisionCommand());

    shootAndLoadButton.whenPressed(new ShootAndLoadCommand());
    shootAndLoadButton.whenReleased(new ShootShooterMagazineAndLoaderStopCommand());

    shootTurretHomeButton.whenPressed(new ShooterTurretCenterCommand());
   // shooterTiltButton.whenPressed(new ShooterTiltGoToSetpointCommand()); 
   //shooterTiltFloorButton.whenPressed(new ShooterTiltFloorCommand());  
    shooterTiltButton.whileActive(new ShooterTestFangsCommand()); 
   //wallFollowButton.whenActive(new DriveFollowWallCommand());
   // visionTestButton.whileActive(new ShooterVisionCommand());
    //shooterWheelControlButton.whenPressed(new ShooterWheelCommand());
    shooterCryBabyButton.whileActive(new ShooterVisionCommand());
    //wallFollowButton.whenActive(new DriveFollowWallCommand());


    //autoDriveForwardButton.whenPressed(new RealSmartAutoDriveForward());
    //autoTurnButton.whenPressed(new RealSmartAutoTurnCommand());
  }

}