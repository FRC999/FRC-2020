/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class BallHandlingSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_TalonSRX hopperMotor = new WPI_TalonSRX(RobotMap.hopperMotorPort);
  WPI_TalonSRX neckMotor = new WPI_TalonSRX(RobotMap.neckMotor);
  //potential solenoid 

  BallHandlingSubsystem(){
    hopperMotor.setNeutralMode(NeutralMode.Brake);
    neckMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * We should look into being able to summon a single ball: first we need to see how
   * grippy the ball handling belts are
   */
  public void runMotors(){
    hopperMotor.set(.2);
    neckMotor.set(.3);//magic numbers because this is a skeleton of a class
  }

  public void stopMotors(){
    hopperMotor.set(0);
    neckMotor.set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    
  }
}
