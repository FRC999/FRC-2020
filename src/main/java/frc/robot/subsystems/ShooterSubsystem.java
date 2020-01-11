package frc.robot.subsystems;


import edu.wpi.first.wpilibj.*;
import frc.robot.RobotMap;
import frc.robot.commands.ManualEndShoot;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShooterSubsystem extends Subsystem {
  /**
   * Creates a new ExampleSubsystem.
   */

public PWMTalonSRX shooterMotor = new PWMTalonSRX(RobotMap.kShooterMotorID);
public PWMTalonSRX panMotor = new PWMTalonSRX(RobotMap.panMotorID);
public PWMTalonSRX tiltMotor = new PWMTalonSRX(RobotMap.tiltMotorID);

  double shooterSpeed = 0.5;

  public void standby(){
    shooterMotor.set(0);
  }

  public void shoot(){
    shooterMotor.set(shooterSpeed);
  }

  public void pan(){

  }

  public void tilt(){

  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

}
