package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

public PWMTalonSRX shooterMotor1 = new PWMTalonSRX(Constants.kShooterMotorID1);
public PWMTalonSRX shooterMotor2 = new PWMTalonSRX(Constants.kShooterMotorID2);
public PWMTalonSRX panMotor = new PWMTalonSRX(Constants.panMotorID);
public PWMTalonSRX tiltMotor = new PWMTalonSRX(Constants.tiltMotorID);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(){
  }

  public void pan(){

  }

  public void tilt(){

  }

}
