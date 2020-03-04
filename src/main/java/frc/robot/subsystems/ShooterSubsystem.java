package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.commands.ShootManuallyCommand;

public class ShooterSubsystem extends Subsystem {

  static WPI_TalonSRX shooterMotorController = new WPI_TalonSRX(RobotMap.shooterWheelMotorControllerID);
  public static WPI_TalonSRX panMotorController = new WPI_TalonSRX(RobotMap.shooterPanMotorControllerID);
  static WPI_TalonSRX tiltMotorController = new WPI_TalonSRX(RobotMap.ShooterTiltMotorControllerID);

  // double shooterSpeed = 0.5;

  NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

  public void configureShooterControllers() {
    shooterMotorController.configFactoryDefault();
    panMotorController.configFactoryDefault();

    if (RobotMap.isFalconBot) {
      panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
      panMotorController.configFeedbackNotContinuous(true, RobotMap.configureTimeoutMs);
    } else {
      panMotorController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
      panMotorController.setSelectedSensorPosition(0);
    }

    tiltMotorController.configFactoryDefault();
    tiltMotorController.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    tiltMotorController.setSelectedSensorPosition(0);
    /*
     * Johnson motors have a quadrature encoder with 178 ticks per revolution. Notes
     * on wiring them to talons: type |motor | breakout board --------------------
     * 5v |brown | red output 1/A |yellow| green output 2/B |green | yellow ground
     * |blue | black
     * 
     */

  }

  public void configurePanMotorControllerForMagic() {
    // Configure the encoders for PID control
    panMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID_PAN,
        RobotMap.configureTimeoutMs);

    /* Set status frame periods to ensure we don't have stale data */
    panMotorController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
    panMotorController.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 20, RobotMap.configureTimeoutMs);

    /* Configure motor neutral deadband */
    panMotorController.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    panMotorController.configPeakOutputForward(+1.0, RobotMap.configureTimeoutMs);
    panMotorController.configPeakOutputReverse(-1.0, RobotMap.configureTimeoutMs);
    panMotorController.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
    panMotorController.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);

    /* FPID Gains for each side of drivetrain */

    panMotorController.config_kP(RobotMap.SLOT_0, RobotMap.P_PAN, RobotMap.configureTimeoutMs);
    panMotorController.config_kI(RobotMap.SLOT_0, RobotMap.I_PAN, RobotMap.configureTimeoutMs);
    panMotorController.config_kD(RobotMap.SLOT_0, RobotMap.D_PAN, RobotMap.configureTimeoutMs);
    panMotorController.config_kF(RobotMap.SLOT_0, RobotMap.F_PAN, RobotMap.configureTimeoutMs);
    panMotorController.config_IntegralZone(RobotMap.SLOT_0, RobotMap.Izone_PAN, RobotMap.configureTimeoutMs);
    panMotorController.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
    panMotorController.configAllowableClosedloopError(RobotMap.SLOT_0, 0, RobotMap.configureTimeoutMs);

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */
    panMotorController.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);

    /* Motion Magic Configurations */
    /**
     * Need to replace numbers with real measured values for acceleration and cruise
     * vel.
     */

    panMotorController.configMotionAcceleration(RobotMap.panAcceleration, RobotMap.configureTimeoutMs);
    panMotorController.configMotionCruiseVelocity(RobotMap.panCruiseVelocity, RobotMap.configureTimeoutMs);
    panMotorController.configMotionSCurveStrength(RobotMap.smoothing);

  } // End configureDriveTrainControllersForSimpleMagic

  public int getPanEncoder() {
    return panMotorController.getSelectedSensorPosition();
  }

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

  public  double deadbandPan(double pan) {
    if (Math.abs(pan) >= RobotMap.deadbandZ){
      if (pan > 0) {
        pan = (pan - RobotMap.deadbandZ) /(1-RobotMap.deadbandZ); 
      }
      else{
        pan = (pan + RobotMap.deadbandZ) /(1-RobotMap.deadbandZ);  
      }
    }
    else {
      pan = 0;
    } 
    return pan;
  }


  public void pan(double pan) {
    panMotorController.set(ControlMode.PercentOutput, deadbandPan(pan));
  }

  /**
   * gets the x-value of the center of the object the camera is looking at. 640 is
   * the maximum; if it returns 1000, the pi is not posting to networktables.
   */
  public double getX() {
    return networkTableInstance.getTable("TestTable/PI").getEntry("X").getDouble(1000);// 640 is the maximum;
  }

  /**
   * gets the y-value of the center of the object the camera is looking at. 480 is
   * the maximum; if it returns 1000, the pi is not posting to networktables.
   */
  public double getY() {
    return networkTableInstance.getTable("TestTable/PI").getEntry("Y").getDouble(1000);// 480 is the maximum;
  }

  /**
   * tests whether the current x-value of the object the robot sees is in the
   * center of its field of view. Outputs the difference (current value - 320).
   */
  public double differenceFromMiddleX() {
    return Math.abs((getX() - (RobotMap.shooterXResolution / 2)));
  }

  /**
   * tests whether the current y-value of the object the robot sees is in the
   * center of its field of view. Outputs the difference (current value - 240).
   */
  public double differenceFromMiddleY() {
    return (getY() - (RobotMap.shooterXResolution / 2));
  }

  public boolean getCenteredX() {
    boolean retVal = false;
    if (Math.abs(differenceFromMiddleX()) <= RobotMap.shooterResolutionAcceptableError) {
      retVal = true;
    }
    return retVal;
  }

  public String whichSide() {
    String state = "";
    if (!getCenteredX() && getX() != 1000) {
      if (getX() <= ((RobotMap.shooterXResolution / 2) - (RobotMap.shooterResolutionAcceptableError))) { // 310
        state = "Left";
      } else if (getX() >= ((RobotMap.shooterXResolution / 2) + (RobotMap.shooterResolutionAcceptableError))) { // 330
        state = "Right";
      } else {
        state = "Center";
      }
    }
    return state;
  }

  public void centerShooter() {
    switch (whichSide()) {
      case "Left": {
        panMotorController.set(ControlMode.MotionMagic, Math.round(
            getPanEncoder() - (differenceFromMiddleX() / RobotMap.pixelsPerDegreeX * RobotMap.encoderTicksPerDegreeX)));
      }
        break;

      case "Center": {
        panMotorController.set(ControlMode.PercentOutput, 0);
      }
        break;

      case "Right": {
        panMotorController.set(ControlMode.MotionMagic, Math.round(
            getPanEncoder() + (differenceFromMiddleX() / RobotMap.pixelsPerDegreeX * RobotMap.encoderTicksPerDegreeX)));
      }
        break;

      default: {
        panMotorController.set(ControlMode.PercentOutput, 0);
      }
    }
  }

//Start of work on the FANGS
  Boolean fangsActivated = false;

public void configureTiltMotorControllerForMagic(){

  // Configure the encoders for PID control
  tiltMotorController.configSelectedFeedbackSensor(FeedbackDevice.Analog, RobotMap.PID_TILT,	RobotMap.configureTimeoutMs);

  /* Set status frame periods to ensure we don't have stale data */
  tiltMotorController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
  tiltMotorController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, RobotMap.configureTimeoutMs);

  /* Configure motor neutral deadband */
  tiltMotorController.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);

  /**
  * Max out the peak output (for all modes).  
  * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
  */
  tiltMotorController.configPeakOutputForward(+1.0, RobotMap.configureTimeoutMs);
  tiltMotorController.configPeakOutputReverse(-1.0, RobotMap.configureTimeoutMs);
  tiltMotorController.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
  tiltMotorController.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);

  /* FPID Gains for each side of drivetrain */

  tiltMotorController.config_kP(RobotMap.SLOT_0, RobotMap.P_PAN, RobotMap.configureTimeoutMs);
  tiltMotorController.config_kI(RobotMap.SLOT_0, RobotMap.I_PAN, RobotMap.configureTimeoutMs);
  tiltMotorController.config_kD(RobotMap.SLOT_0, RobotMap.D_PAN, RobotMap.configureTimeoutMs);
  tiltMotorController.config_kF(RobotMap.SLOT_0, RobotMap.F_PAN, RobotMap.configureTimeoutMs);
  tiltMotorController.config_IntegralZone(RobotMap.SLOT_0, RobotMap.Izone_TILT, RobotMap.configureTimeoutMs);
  tiltMotorController.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
  tiltMotorController.configAllowableClosedloopError(RobotMap.SLOT_0, 0, RobotMap.configureTimeoutMs);

  /**
  * 1ms per loop.  PID loop can be slowed down if need be.
  * For example,
  * - if sensor updates are too slow
  * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
  * - sensor movement is very slow causing the derivative error to be near zero.
  */
  tiltMotorController.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);

  /* Motion Magic Configurations */
  /**Need to replace numbers with real measured values for acceleration and cruise vel. */

  tiltMotorController.configMotionAcceleration(RobotMap.tiltAcceleration, RobotMap.configureTimeoutMs);
  tiltMotorController.configMotionCruiseVelocity(RobotMap.tiltCruiseVelocity, RobotMap.configureTimeoutMs);
  tiltMotorController.configMotionSCurveStrength(RobotMap.smoothing);

  } 
  
  public int getTiltPot() {
    return tiltMotorController.getSelectedSensorPosition();
    //1024 units per rotation
  }

  public void tiltStandby() {
    tiltMotorController.set(ControlMode.PercentOutput, 0);
  }

  public void tiltGoToSetpoint() {
    configureTiltMotorControllerForMagic();
    tiltMotorController.set(ControlMode.MotionMagic, 500);

  }


  public void tiltFangDeployToggle(){
    if (fangsActivated==false)
    {
    tiltMotorController.set(ControlMode.MotionMagic, RobotMap.tiltFangsUpperLimit);
    fangsActivated = true;
    }

    else if (fangsActivated==true)
    {
      tiltMotorController.set(ControlMode.MotionMagic, RobotMap.tiltFangsLowerLimit);
      fangsActivated = false;
    }
  } 

  public void manualAimTiltFangs(){
    double tiltValue = ((Robot.oi.leftJoystick.getThrottle()*-1) + 1) / 2;
    double output = tiltValue*RobotMap.shooterTiltMotorTicksPerRotation;

    if (fangsActivated==true)
    {
    tiltMotorController.set(ControlMode.MotionMagic, output);
    }
    //Use a constant for the activated position of the encoders and then add to it.  
  }

  public void testTiltFangs(){
    //System.out.println("Testing");
    double maxSpeed = 0; //sets motorspeed to 0 if tilt is against stops
    if (getTiltPot() >= RobotMap.tiltFangsLowerLimit &&  getTiltPot() <= RobotMap.tiltFangsUpperLimit) {
      maxSpeed = 0.25;
    }
    double output = (Robot.oi.leftJoystick.getThrottle()*1) * maxSpeed;
    Robot.smartDashboardSubsystem.updateShooterValues();
   // System.out.println(output);
    tiltMotorController.set(ControlMode.PercentOutput, output);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

}