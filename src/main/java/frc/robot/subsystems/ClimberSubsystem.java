/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class ClimberSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  static WPI_VictorSPX climberMotor = new WPI_VictorSPX(RobotMap.climberMotorID);
  static Solenoid climberSolenoid1 = new Solenoid(RobotMap.climberSolenoid1Channel);
  static Solenoid climberSolenoid2 = new Solenoid(RobotMap.climberSolenoid2Channel);

    double climbSpeed = 0.5;

  public void extend(){
      climberSolenoid1.set(true);
      climberSolenoid2.set(true);
  }

  public void retract(){
    climberSolenoid1.set(false);
    climberSolenoid2.set(false);
}

  public void climb(){
    climberMotor.set(climbSpeed);
  }

  public void standby(){
    climberMotor.set(0);
    }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
