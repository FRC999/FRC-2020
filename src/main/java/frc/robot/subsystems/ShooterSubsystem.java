package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * TODO: write a method to check the encoder and figure out whch way to go to
 * get to zero in the shortest time
 */
public class ShooterSubsystem extends Subsystem {

  static WPI_TalonSRX shooterMotorController = new WPI_TalonSRX(RobotMap.shooterWheelMotorController);
  static WPI_TalonSRX panMotorController = new WPI_TalonSRX(RobotMap.shooterPanMotorController);
  static WPI_TalonSRX tiltMotorController = new WPI_TalonSRX(RobotMap.ShooterTiltMotorController);
  // public SensorCollection turretEncoder;

  // double shooterSpeed = 0.5;

  NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

  /**
   * This is to compensate for the fact that the zero point of the turret is
   * unknown. This adjusts the zero point to point to the back of the robot.
   */
  int adjustedTurretPosition;

  public void configureShooterControllers() {
    // this.turretEncoder = turretEncoder;
    shooterMotorController.configFactoryDefault();
    panMotorController.configFactoryDefault();
    tiltMotorController.configFactoryDefault();
    panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    panMotorController.configFeedbackNotContinuous(true, RobotMap.configureTimeoutMs);
    // panMotorController.turretEncoder.getPulseWidthRiseToFallUs()
    // tiltMotorController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  /**
   * Update the 'adjusted' turret position: the position we use to compenstate for
   * the fact that the zero point of the real turret may be in an awkward
   * location.
   */
  public void updateAdjustedTurretPosition() {
    adjustedTurretPosition = adjustTurretPosition(0);// TODO: get a method call to get the real pos.
  }

  /**
   * Convert from an absolute position (given by the encoder) to an adjusted
   * position (what we use)
   * 
   * @param absolute the encoder ticks in an absolute scale
   * @return the encoder ticks in the adjusted scale
   */
  public int adjustTurretPosition(int absolute) {
    int relative = absolute - RobotMap.shooterPanMotorEncoderOffset;
    if (relative < 0) {
      relative += RobotMap.shooterPanMotorEncoderTicksPerRotation + 1;
    }
    return relative;
  }

  /**
   * Convert from the relative/adjusted position (what we use) to the absolute
   * position that the motor takes
   * 
   * @param relative the encoder ticks in a relative scale
   * @return encoder ticks in an absolute scale
   */
  public int deAdjustTurretPosition(int relative) {
    int absolute = relative + RobotMap.shooterPanMotorEncoderOffset;
    if (absolute > RobotMap.shooterPanMotorEncoderTicksPerRotation) {
      absolute -= RobotMap.shooterPanMotorEncoderTicksPerRotation + 1;
    }
    return absolute;
  }

  public int getPanEncoder() {
    return panMotorController.getSelectedSensorPosition();
  }

  public void zeroShooterEncoders() {
    panMotorController.setSelectedSensorPosition(0);
  }

  // TODO: Find out why Dr. Wertz commented out this stuff
  // public int getTiltEncoder() {
  // return tiltMotorController.getSelectedSensorPosition();
  // }

  /** stops the shooter motor. */
  public void standby() {
    shooterMotorController.set(ControlMode.PercentOutput, 0);
  }

  public void shoot(double shooterSpeed) {
    shooterMotorController.set(ControlMode.PercentOutput, shooterSpeed);
  }

  public void pan(double pan) {
    panMotorController.set(ControlMode.PercentOutput, pan);
  }

  // public void tilt(double tilt) {
  // tiltMotorController.set(ControlMode.PercentOutput, tilt);
  // }
  /**
   * gets the x-value of the center of the object the camera is looking at. 640 is
   * the maximum; if it returns 1000, the pi is not posting to NetworkTables.
   */
  public double getX() {
    return networkTableInstance.getTable("TestTable/PI").getEntry("X").getDouble(1000);// 640 is the maximum;
  }

  /**
   * gets the y-value of the center of the object the camera is looking at. 480 is
   * the maximum; if it returns 1000, the pi is not posting to NetworkTables.
   */
  public double getY() {
    return networkTableInstance.getTable("TestTable/PI").getEntry("Y").getDouble(1000);// 480 is the maximum;
  }

  /**
   * tests whether the current x-value of the object the robot sees is in the
   * center of its field of view. Outputs the difference (current value - 320).
   */
  public double getDifferenceFromMiddleX() {
    return (getX() - (RobotMap.shooterXResolution / 2));
  }

  /**
   * tests whether the current y-value of the object the robot sees is in the
   * center of its field of view. Outputs the difference (current value - 240).
   */
  public double getDifferenceFromMiddleY() {
    return (getY() - (RobotMap.shooterXResolution / 2));
  }

  public boolean getCenteredX() {
    boolean retVal = false;
    if (Math.abs(getDifferenceFromMiddleX()) <= RobotMap.shooterResolutionAcceptableError) {
      retVal = true;
    }
    return retVal;
  }

  public int getWhichWayToTurnToGetToZero() {
    int retVal = 0;
    if (getPanEncoder() <= RobotMap.shooterPanMotorEncoderTicksPerRotation / 2) {
      retVal = -1;
    } else {
      retVal = 1;
    }
    return retVal;
  }

  public int getWhichWayToTurnToGetToAngle(double encoderTicksRequested) {
    int retVal = 0;
    double target = encoderTicksRequested;
    if (encoderTicksRequested > RobotMap.shooterPanMotorEncoderTicksBeforeRollover) {
      target = encoderTicksRequested - RobotMap.shooterPanMotorEncoderTicksBeforeRollover;
    } else if (encoderTicksRequested < 0) {
      target = RobotMap.shooterPanMotorEncoderTicksBeforeRollover + encoderTicksRequested;
    }

    if ((getPanEncoder() <= 20 + RobotMap.shooterPanMotorEncoderTicksPerRotation / 2)
        && (getPanEncoder() >= RobotMap.shooterPanMotorEncoderTicksPerRotation / 2 - 20)) {
      if (target > getPanEncoder()) {
        retVal = 1;
      } else {
        retVal = -1;
      }
    } else if (target > getPanEncoder()) {
      if (target > getPanEncoder() + RobotMap.shooterPanMotorEncoderTicksPerRotation / 2) {
        retVal = -1;
      } else {
        retVal = 1;
      }
    } else if (target < getPanEncoder()) {
      if (target > getPanEncoder() - RobotMap.shooterPanMotorEncoderTicksPerRotation / 2) {
        retVal = -1;
      } else {
        retVal = 1;
      }
    }

    return retVal;
  }

  public double getHeadingDegreesFromPanEncoderValue() {
    double retVal = 0;
    retVal = getPanEncoder() * (360. / RobotMap.shooterPanMotorEncoderTicksPerRotation);
    return retVal;
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

}
