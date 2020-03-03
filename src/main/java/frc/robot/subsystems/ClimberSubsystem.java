/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class ClimberSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static WPI_VictorSPX climberMotorController = new WPI_VictorSPX(RobotMap.climberMotorControllerID);
  static DoubleSolenoid climberSolenoid = new DoubleSolenoid(RobotMap.climberSolenoidForwardChannel, RobotMap.climberSolenoidReverseChannel);

  //double climbSpeed = 0.5;
  //TODO: Figure out why we have a solenoid and a motor
  public void extend() {
    climberSolenoid.set(Value.kForward);
  }

  public void retract() {
    climberSolenoid.set(Value.kReverse);
  }

  public void climb(double climbSpeed) {
    climberMotorController.set(ControlMode.PercentOutput, climbSpeed);
  }

  public void standby() {
    climberMotorController.set(ControlMode.PercentOutput, 0);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
