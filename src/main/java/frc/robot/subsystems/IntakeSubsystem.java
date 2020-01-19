/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;


public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static WPI_VictorSPX intakeMotor1 = new WPI_VictorSPX(RobotMap.intakeMotor1ID);
  static WPI_VictorSPX intakeMotor2 = new WPI_VictorSPX(RobotMap.intakeMotor2ID);

  double intakeSpeed = 0.5;

  public void standby(){
    intakeMotor1.set(0);
    intakeMotor2.set(0);
  }

  public void intakeIn(){
    intakeMotor1.set(intakeSpeed);
    intakeMotor2.set(intakeSpeed);
  }

  public void intakeReverse(){
    intakeMotor1.set(-intakeSpeed);
    intakeMotor2.set(-intakeSpeed);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
