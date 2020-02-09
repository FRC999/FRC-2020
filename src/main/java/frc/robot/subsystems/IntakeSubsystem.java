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

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;


public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static WPI_VictorSPX intakeMotor1Controller = new WPI_VictorSPX(RobotMap.intakeMotor1Controller);
  static WPI_VictorSPX intakeMotor2Controller = new WPI_VictorSPX(RobotMap.intakeMotor2Controller);

  public static DoubleSolenoid intakeSolenoid ;//= new DoubleSolenoid(RobotMap.IntakeSolenoidForwardChannel,RobotMap.IntakeSolenoidReverseChannel);

  //double intakeSpeed = 0.5;

  public void standby(){
    intakeMotor1Controller.set(ControlMode.PercentOutput, 0);
    intakeMotor2Controller.set(ControlMode.PercentOutput, 0);
  }

  public void intakeIn(double intakeSpeed){
    intakeMotor1Controller.set(ControlMode.PercentOutput, intakeSpeed);
    intakeMotor2Controller.set(ControlMode.PercentOutput, intakeSpeed);
  }

  /** sets the intake solenoid (piston controller) to either its forward, reverse, or off states, using the enum DoubleSolenoid.Value's states kForward, kReverse, and kOff.*/
  public void SetIntakeSolenoid(DoubleSolenoid.Value val) {
    intakeSolenoid.set(val);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
