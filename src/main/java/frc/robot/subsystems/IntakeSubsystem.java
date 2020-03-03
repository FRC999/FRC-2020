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


public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static WPI_VictorSPX intakeMotorController = new WPI_VictorSPX(RobotMap.intakeMotorControllerID);
  static WPI_VictorSPX magazineMotorController = new WPI_VictorSPX(RobotMap.magazineMotorControllerID);
  static WPI_VictorSPX loaderMotor1Controller = new WPI_VictorSPX(RobotMap.loaderMotor1ControllerID);
  static WPI_VictorSPX loaderMotor2Controller = new WPI_VictorSPX(RobotMap.loaderMotor2ControllerID);

  public static DoubleSolenoid intakeSolenoid = new DoubleSolenoid(RobotMap.IntakeSolenoidForwardChannel,RobotMap.IntakeSolenoidReverseChannel);


  public void standby(){
    intakeMotorController.set(ControlMode.PercentOutput, 0);
    magazineMotorController.set(ControlMode.PercentOutput, 0);
    loaderMotor1Controller.set(ControlMode.PercentOutput, 0);
    loaderMotor2Controller.set(ControlMode.PercentOutput, 0);
  }

  public void intake(double motorSpeed){
    intakeMotorController.set(ControlMode.PercentOutput, motorSpeed);
  }

  public void magazine(double motorSpeed){
    magazineMotorController.set(ControlMode.PercentOutput, motorSpeed);
  }

  public void loader(double motorSpeed){
    loaderMotor1Controller.set(ControlMode.PercentOutput, motorSpeed);
    loaderMotor2Controller.set(ControlMode.PercentOutput, motorSpeed);
  }

  public void IntakeUp() {
    intakeSolenoid.set(Value.kForward);
  }

  public void IntakeDown() {
    intakeSolenoid.set(Value.kReverse);
  }

  /** sets the intake solenoid (piston controller) to either its forward, reverse, or off states, using the enum DoubleSolenoid.Value's states kForward, kReverse, and kOff.*/
  public void SetIntakeSolenoid(DoubleSolenoid.Value val) {
    intakeSolenoid.set(val);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }
}
