/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.RealSmartAutoCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystemBase;
import frc.robot.subsystems.TalonDriveSubsystem;
import frc.robot.subsystems.FalconDriveSubsystem;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.UltrasonicSensorSubsystem;
import edu.wpi.first.networktables.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  NetworkTable table;
  public static DriveSubsystemBase driveSubsystem;
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public static SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  public static NavXSubsystem navXSubsystem = new NavXSubsystem();
  public static UltrasonicSensorSubsystem ultrasonicSubsystem = new UltrasonicSensorSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static ControlPanelSubsystem controlPanelSubsystem = new ControlPanelSubsystem();
  public static ShuffleboardSubsystem shuffleBoardSubsystem = new ShuffleboardSubsystem();

  
  public boolean TestBool = false;
  public static OI oi;
  Command autonomousCommand;

  // Sendable choosers belowP
  SendableChooser<Command> sendableCommandChooser = new SendableChooser<Command>();
  SendableChooser<String> sendableStringChooser = new SendableChooser<String>();
  SendableChooser<Integer> sendableIntegerChooser = new SendableChooser<Integer>();
  SendableChooser<Double> sendableDoubleChooser = new SendableChooser<Double>();
  // End sendable choosers

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    
    //Set up shuffleboard
    shuffleBoardSubsystem.setupShuffleboard();


    NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    ntInst.startClientTeam(999);
    NetworkTable table = ntInst.getTable("TestTable");
    NetworkTableEntry testEntry = table.getEntry("test");
    testEntry.setDouble(10.5);
    System.out.println("Hit robotInit");

    DigitalInput falconBotSwitch = new DigitalInput(RobotMap.falconBotSwitchPortNumber);
    RobotMap.isFalconBot = !falconBotSwitch.get();
    System.out.println("falconBotSwitch = "+ RobotMap.isFalconBot);
    if(RobotMap.isFalconBot){
      driveSubsystem = new FalconDriveSubsystem();
      // the IAmFalconBot method reset some RobotMap constants for the FalconBot chassis
      // but the call to it was moved into the FalconDriveSubsystem constructor
      System.out.println("We're a FALCON");
    }
    else{
      driveSubsystem = new TalonDriveSubsystem();
      System.out.println("We're a TALON");
    }
    driveSubsystem.setDefaultCommand(new DriveManuallyCommand());
    falconBotSwitch.close();

    sendableCommandChooser.setDefaultOption("Default Auto", new RealSmartAutoCommand());
    sendableCommandChooser.addOption("Really Smart Auto", new RealSmartAutoCommand());

    Robot.driveSubsystem.resetDriveTrainControllers();

    // after testing run only the second configure method
    Robot.driveSubsystem.configureDriveTrainControllersForSimpleMagic();

    Robot.driveSubsystem.zeroDriveEncoders();
    Robot.driveSubsystem.driveTrainBrakeMode();
    Robot.navXSubsystem.zeroYaw();
    Robot.shooterSubsystem.configureShooterControllers();
    Robot.shooterSubsystem.configurePanMotorControllerForMagic();
    //Robot.shooterSubsystem.zeroShooterEncoders();
    //Robot.controlPanelSubsystem.resetMotorController();
    

    oi = new OI();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    smartDashboardSubsystem.updateAllDisplays();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    driveSubsystem.DriveTrainCoastMode();
    //controlPanelSubsystem.stopTalon();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    
    autonomousCommand = sendableCommandChooser.getSelected();
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */
    driveSubsystem.driveTrainBrakeMode();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {

    driveSubsystem.driveTrainBrakeMode();
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    smartDashboardSubsystem.updateNavXValues();
    smartDashboardSubsystem.updateEncoderValue();
    
    //controlPanelSubsystem.putSeenColor();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
  }
}
