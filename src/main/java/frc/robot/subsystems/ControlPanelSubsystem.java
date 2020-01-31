package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.robot.RobotMap;

public class ControlPanelSubsystem extends Subsystem {
  // TODO: Find stuff for JE-PLG-149 motors so they can be used here

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  private WPI_TalonSRX diskSpinnerTalon = new WPI_TalonSRX(RobotMap.diskSpinnerMotorID);
  private DoubleSolenoid diskSpinnerSolenoid= new DoubleSolenoid(RobotMap.ColorWheelSolenoidForwardChannel,RobotMap.ColorWheelSolenoidReverseChannel);

  private PanelColors targetColor;
  private boolean receivedGameColor = false;
  private boolean colorsAligned = false;
  private Color currentColor;
  private PanelColors suspectedColor;

  static final double toleranceSize = .01;// tolerance size (in percent)
  private boolean wasAligned = false;
  private double spinSpeed = 0.5;

  public  void resetMotorController() {
    diskSpinnerTalon.configFactoryDefault();
    diskSpinnerTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }
  public int readEncoder() {
    return diskSpinnerTalon.getSelectedSensorPosition();
  }

public void zeroEncoder() {
  diskSpinnerTalon.setSelectedSensorPosition(0);
}

  public void putSeenColor() {
    updateColorState();
    SmartDashboard.putNumber("Spotted Color: Red", currentColor.red * 255);
    SmartDashboard.putNumber("Spotted Color: Green", currentColor.green * 255);
    SmartDashboard.putNumber("Spotted Color: Blue", currentColor.blue * 255);
    SmartDashboard.putNumber("Spotted Distance: ", getProximity());
    if (suspectedColor != null) {
      SmartDashboard.putString("SuspectedColor: ", suspectedColor.toString());
    }
  }
  
  public boolean hasReceivedGameColor(){
    return receivedGameColor;
  }

  // NOTE: Will be implementing Rotation Control as a command
  public void spinToTargetColor() {
    if (checkColorAlignment()) {
      diskSpinnerTalon.setNeutralMode(NeutralMode.Brake);
      diskSpinnerTalon.set(0);
      // TODO: Use actual motion profiling (note: maybe not, will definitely be very hard)
      // Use position MotionMagic... It is very good for this type of application
    } else {
      diskSpinnerTalon.set(spinSpeed);
    }

    if (wasAligned != colorsAligned) {// if alignment has changed since last check...
      spinSpeed *= -.8; // Switch spin directions and decrease speed: we overshot
    }
    wasAligned = colorsAligned;
  }

  /**
   * Sets the value of this subsystem's DoubleSolenoid: kForward, kReverse, or
   * kOff. These are values of the enum DoubleSolenoid.Value.
   */
  public void setSolenoid(DoubleSolenoid.Value val) {
    diskSpinnerSolenoid.set(val);

  }

  /**
   * Update the color that we should be reading:
   */
  public void updateColorState() {
    targetColor = getGameTargetColor();
    currentColor = colorSensor.getColor();
    suspectedColor = getSuspectedColor(currentColor);

  }

  /**
   * Converts from 'color' to 'PanelColor': assumes no overlap
   * 
   * @param colour the color to check
   * @return the determined color (may be null)
   */
  public PanelColors getSuspectedColor(Color colour) { // Mr. Wertz approved -CMM
    for (PanelColors isItMe : PanelColors.values()) {
      if (isItMe.withinTolerance(colour.red, colour.green, colour.blue)) {
        return isItMe;
      }
    }
    return null;
  }

  /**
   * Checks whether the colors are aligned: null checked
   * 
   * @return true if aligned: false otherwise
   */
  public boolean checkColorAlignment() {
    updateColorState();

    // Quick null check:
    if (targetColor == null || currentColor == null) {
      return colorsAligned = false;
    }
    return targetColor.withinTolerance(currentColor.red, currentColor.blue, currentColor.green);

  }

  /**
   * Get what the game would like our sensor to read
   * 
   * @return color that our sensor should read
   */
  public PanelColors getGameTargetColor() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      receivedGameColor = true;
      switch (gameData.charAt(0)) {
      case 'B':
        return PanelColors.red;
      case 'G':
        return PanelColors.yellow;
      case 'R':
        return PanelColors.blue;
      case 'Y':
        return PanelColors.green;
      default:
        return null; // corrupt data! uh oh!
      }
    } else {
      receivedGameColor = false;
      return null;// we haven't gotten it yet
    }
  }

  /**
   * Returns the approximate distance from the color scanner (At a high value,
   * color may not be accurate)
   * 
   * @return distance from color scanner from 0-2047
   */
  public int getProximity() {
    return colorSensor.getProximity();
  }

  enum PanelColors {
    blue(0, 1, 1), green(0, 1, 0), red(1, 0, 0), yellow(1, 0, 0);
    // TODO: Find more accurate values, also reformat
    // Maybe use HSV for tolerance check?

    // numbers gotten from game manual + online converter
    final private double redVal, greenVal, blueVal;

    PanelColors(int r, int g, int b) {
      redVal = r;
      greenVal = g;
      blueVal = b;
    }

    Color getColor() {
      return new Color(redVal, greenVal, blueVal);
    }

    boolean withinTolerance(double r, double g, double b) {
      if (Math.abs(redVal - r) < toleranceSize && Math.abs(greenVal - g) < toleranceSize
          && Math.abs(blueVal - b) < toleranceSize) {
        return true;
      }
      return false;
    }

  }

  @Override
  public void initDefaultCommand() {

  }
}