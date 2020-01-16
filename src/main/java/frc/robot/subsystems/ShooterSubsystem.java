package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShooterSubsystem extends Subsystem {
  /**
   * Creates a new ExampleSubsystem.
   */

  static WPI_TalonSRX shooterMotor = new WPI_TalonSRX(RobotMap.kShooterMotorID);
  static WPI_TalonSRX panMotor = new WPI_TalonSRX(RobotMap.panMotorID);
  static WPI_TalonSRX tiltMotor = new WPI_TalonSRX(RobotMap.tiltMotorID);

  double shooterSpeed = 0.5;

  public void standby() {
    shooterMotor.set(0);
  }

  public void shoot() {
    shooterMotor.set(shooterSpeed);
  }

  public void pan() {

  }

  public void tilt() {

  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

}
