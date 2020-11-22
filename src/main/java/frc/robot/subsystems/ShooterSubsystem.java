package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.NetworkTablesSubsystem;
public class ShooterSubsystem extends Subsystem {

  static WPI_TalonSRX shooterMotorController = new WPI_TalonSRX(RobotMap.shooterWheelMotorControllerID);
  public static WPI_TalonSRX panMotorController = new WPI_TalonSRX(RobotMap.shooterPanMotorControllerID);
  static WPI_TalonSRX tiltMotorController = new WPI_TalonSRX(RobotMap.ShooterTiltMotorControllerID);
 // public SensorCollection turretEncoder;

  // double shooterSpeed = 0.5;

  public void configureShooterControllers(){
    //this.turretEncoder = turretEncoder;
    shooterMotorController.configFactoryDefault();
    panMotorController.configFactoryDefault();

    if(RobotMap.isFalconBot) {
      //this.turretEncoder = turretEncoder;
      panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
      panMotorController.configFeedbackNotContinuous(true, RobotMap.configureTimeoutMs);
      // panMotorController.turretEncoder.getPulseWidthRiseToFallUs();
    }
    else {
      panMotorController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
      panMotorController.setSelectedSensorPosition(0); 
    }

    tiltMotorController.configFactoryDefault();
    tiltMotorController.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    tiltMotorController.setSelectedSensorPosition(0);
     /*Johnson motors have a quadrature encoder with 178 ticks per revolution. Notes on wiring them to talons:
      type   |motor | breakout board
             --------------------
          5v |brown | red
  output 1/A |yellow| green
  output 2/B |green | yellow
      ground |blue  | black
    
    */

  }

  // replace with configure controllers for aux closed loop PID when ready
  public void configurePanMotorControllerForMagic(){

	  // Configure the encoders for PID control
	  panMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID_PAN,	RobotMap.configureTimeoutMs);
	
	  /* Set status frame periods to ensure we don't have stale data */
	  panMotorController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.configureTimeoutMs);
	  panMotorController.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 20, RobotMap.configureTimeoutMs);

	  /* Configure motor neutral deadband */
	  panMotorController.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);

    /**
	  * Max out the peak output (for all modes).  
	  * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
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
	  * 1ms per loop.  PID loop can be slowed down if need be.
	  * For example,
	  * - if sensor updates are too slow
	  * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
	  * - sensor movement is very slow causing the derivative error to be near zero.
	  */
	  panMotorController.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);

    /* Motion Magic Configurations */
    /**Need to replace numbers with real measured values for acceleration and cruise vel. */

    panMotorController.configMotionAcceleration(RobotMap.panAcceleration, RobotMap.configureTimeoutMs);
	  panMotorController.configMotionCruiseVelocity(RobotMap.panCruiseVelocity, RobotMap.configureTimeoutMs);
    panMotorController.configMotionSCurveStrength(RobotMap.smoothing);

  } // End configureDriveTrainControllersForSimpleMagic



  public int getpanEncoder() {
    return panMotorController.getSelectedSensorPosition();
  }

 

  //public int gettiltEncoder() {
    // return tiltMotorController.getSelectedSensorPosition();
  //}

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


  
  public void setXCoords() {
    ntInst.getTable("TestTable/PI").getEntry("X").setDouble();
  }
  public void setYCoords() {
    ntInst.getTable("TestTable/PI").getEntry("Y").setDouble();
  }

  /** gets the x-value of the center of the object the camera is looking at. 640 is the maximum; if it returns 1000, the pi is not posting to networktables.*/
  public double getX() {
   return ntInst.getTable("TestTable/PI").getEntry("X").getDouble(1000);// 640 is the maximum; 
  }

  /** gets the y-value of the center of the object the camera is looking at.  480 is the maximum; if it returns 1000, the pi is not posting to networktables.*/
  public double getY() {
    return ntInst.getTable("TestTable/PI").getEntry("Y").getDouble(1000);// 480 is the maximum; 
  }

  /**tests whether the current x-value of the object the robot sees is in the center of its field of view. Outputs the difference (current value - 320). */
  public double differenceFromMiddleX() {
    return Math.abs((getX() -(RobotMap.shooterXResolution/2)));
  }
 
  /**tests whether the current y-value of the object the robot sees is in the center of its field of view. Outputs the difference (current value - 240). */
  public double differenceFromMiddleY() {
    return (getY() - ( RobotMap.shooterXResolution/2));
  }

  public boolean getCenteredX() {
    boolean retVal = false;
    if(Math.abs(differenceFromMiddleX()) <= RobotMap.shooterResolutionAcceptableError) {
      retVal = true;
    }
    return retVal;
  }

  public String whichSide() {
    String state = "";
    if(!getCenteredX() && getX() != 1000) {
      if (getX() <= ((RobotMap.shooterXResolution/2)-(RobotMap.shooterResolutionAcceptableError))) { //310
        state = "Left";
      } else if (getX() >= ((RobotMap.shooterXResolution/2)+(RobotMap.shooterResolutionAcceptableError))) { //330
        state = "Right";
      }
      else {
        state = "Center";
      }
    }
    return state;
  }



  public void centerShooter() {
    switch (whichSide()) {
      case "Left" : {
        panMotorController.set(ControlMode.MotionMagic, Math.round(getpanEncoder() - (differenceFromMiddleX() / RobotMap.pixelsPerDegreeX * RobotMap.encoderTicksPerDegreeX)));
        //panMotorController.set(RobotMap.shooterPanSpeed);
        // System.out.println("TARGET LEFT OF CENTER");
      }
      break;

      case "Center" : {
        panMotorController.set(ControlMode.PercentOutput, 0);
        //panMotorController.set(0);
        // System.out.println("TARGET IN CENTER " + getX() +" PanEncoder " +Math.round(getpanEncoder() - (differenceFromMiddleX() / RobotMap.pixelsPerDegreeX * RobotMap.encoderTicksPerDegreeX))) ;
      }
      break;

      case "Right" : {
        panMotorController.set(ControlMode.MotionMagic, Math.round(getpanEncoder() + (differenceFromMiddleX() / RobotMap.pixelsPerDegreeX * RobotMap.encoderTicksPerDegreeX)));
        //panMotorController.set((RobotMap.shooterPanSpeed)*-1);
        // System.out.println("TARGET RIGHT OF CENTER");
      }
      break;
      
      default : {
        panMotorController.set(ControlMode.PercentOutput, 0);
        // System.out.println("DEFAULT");
      }
    }
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

}