package frc.robot.subsystems;


import edu.wpi.first.wpilibj.*;
import frc.robot.RobotMap;
import frc.robot.commands.ManualShooterCommand;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShooterSubsystem extends Subsystem {
  /**
   * Creates a new ExampleSubsystem.
   */

public PWMTalonSRX shooterMotor = new PWMTalonSRX(RobotMap.kShooterMotorID);
public PWMTalonSRX panMotor = new PWMTalonSRX(RobotMap.panMotorID);
public PWMTalonSRX tiltMotor = new PWMTalonSRX(RobotMap.tiltMotorID);

  public void standby(){

  }

  public void shoot(){
  }

  public void pan(){

  }

  public void tilt(){

  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualShooterCommand());
  }

}
