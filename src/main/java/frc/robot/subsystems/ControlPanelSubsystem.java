package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotMap;

public class ControlPanelSubsystem extends Subsystem {
  // TODO: Find stuff for JE-PLG-149 motors so they can be used here

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  private WPI_TalonSRX diskSpinnerTalon = new WPI_TalonSRX(RobotMap.diskSpinnerMotorID);
  private DoubleSolenoid diskSpinnerSolenoid;//= new DoubleSolenoid(RobotMap.ColorWheelSolenoidForwardChannel,RobotMap.ColorWheelSolenoidReverseChannel);

  private PanelColors targetColor;
  private boolean receivedGameColor = false;
  private boolean colorsAligned = false;
  private Color currentColor;
  private PanelColors suspectedColor;

  static final double toleranceSize = .01;// tolerance size (in percent)
  private boolean wasAligned = false;
  private double spinSpeed = 0.5;

  public ControlPanelSubsystem()
  {
    resetMotorController();
  }

  public  void resetMotorController() {
    diskSpinnerTalon.configFactoryDefault();
    diskSpinnerTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    /*Johnson motors have a quadrature encoder with 178 ticks per revolution. Notes on wiring them to talons:
      type   |motor | breakout board
             --------------------
          5v |brown | red
  output 1/A |yellow| green
  output 2/B |green | yellow
      ground |blue  | black
    
    */
    zeroEncoder();
  }
  /** angular position of the control panel motor in encoder ticks. This uses a quadrature encoder, with 178 ticks per revolution. */
  public int readEncoderRaw() {
    return diskSpinnerTalon.getSelectedSensorPosition();
  }
/**  angular position of the control panel motor in revolutions. Converted from raw encoder ticks using the formula
 *  (encoder ticks) * (1 revolution/178 ticks) = the number of revolutions. 
 * Experimentally: 4 revolutions = 712 encoder ticks / 4 = 178
 */
  public double readEncoderRevolutions() {
    return diskSpinnerTalon.getSelectedSensorPosition() * (1/RobotMap.quadratureEncoderTicksPerRev) ;
  }
/** set the value of this subsystem's motor encoder to zero. */
public void zeroEncoder() {
  diskSpinnerTalon.setSelectedSensorPosition(0);
}
/** convert a target value in revolutions for how far we want to spin the control panel 
 * to the number of encoder ticks we need to spin our cylinder to get the control panel there.
 */
public double controlPanelTargetRevolutionsToQuadEncoderTicks(double target) {
/*  angular displacement of gear A / angular displacement of gear B = radius of B / radius of A
theta cyl/theta control = r control / r cyl
theta cyl = (theta control * r control)/r cyl
theta cyl rev * (ticks/rev) = theta cyl ticks
*/
 return (target *RobotMap.controlPanelDiameter * RobotMap.quadratureEncoderTicksPerRev)/(RobotMap.diskSpinnerDiameter);
}
/** Uses this talon's encoder */
public void moveTalonToPosition(double position) {
 // diskSpinnerTalon.set(ControlMode.Position,position);
 diskSpinnerTalon.set(0.5);
 System.out.println(position);
}
/**basically just for test debugging */public void stopTalon() {diskSpinnerTalon.set(0);}

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
 // public void setSolenoid(DoubleSolenoid.Value val) {
  //  diskSpinnerSolenoid.set(val);

  //}

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
    blue(0, 255, 255), green(0, 255, 0), red(255, 0, 0), yellow(255, 0, 0);
    // TODO: Find more accurate values, also reformat
    // Maybe use HSV for tolerance check?

    // numbers gotten from game manual + online converter
    final private int redVal, greenVal, blueVal;

    PanelColors(int r, int g, int b) {
      redVal = r;
      greenVal = g;
      blueVal = b;
    }

    public Color getColor() {
      Color c = new Color(new Color8Bit(redVal, greenVal, blueVal));
      return c;
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