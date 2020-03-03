package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotMap;

public class ControlPanelSubsystem extends Subsystem {
  // TODO: Find stuff for JE-PLG-149 motors so they can be used here

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  private WPI_TalonSRX diskSpinnerTalon = new WPI_TalonSRX(RobotMap.diskSpinnerMotorControllerID);
  //private DoubleSolenoid diskSpinnerSolenoid = new DoubleSolenoid(RobotMap.ColorWheelSolenoidForwardChannel,RobotMap.ColorWheelSolenoidReverseChannel);

  private PanelColors targetColor;
  private boolean receivedGameColor = false;
  private boolean colorsAligned = false;
  private Color currentColor;
  private PanelColors suspectedColor;

  static final double toleranceSize = .05;// tolerance size (in percent)
  private boolean wasAligned = false;
  private double spinSpeed = 0.5;

  public ControlPanelSubsystem()
  {
    //TODO uncomment after testing
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
    return diskSpinnerTalon.getSelectedSensorPosition() * (1./RobotMap.quadratureEncoderTicksPerRev) ;
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
System.out.println("target:" + target);
 double retVal = (target * RobotMap.controlPanelDiameter * RobotMap.quadratureEncoderTicksPerRev)/(RobotMap.diskSpinnerDiameter);
System.out.println(target + " * " + RobotMap.controlPanelDiameter + " * " + RobotMap.quadratureEncoderTicksPerRev + " / " + RobotMap.diskSpinnerDiameter + " = " + retVal);
 return retVal;
}
/** moves the motor at specified power in the direction specified by the sign of the input. 
 * @param power the motor output, 0-1
 * @param position the number whose sign specifies direction
*/
public void moveTalonInDirection(double position, double power) {
 // diskSpinnerTalon.set(ControlMode.Position,position);

 diskSpinnerTalon.set(power * Math.signum(position));

}
/**basically just for test debugging */public void stopTalon() {diskSpinnerTalon.set(0);}
Color lastSeenColor;
/**gets the raw color that the color sensor detects. Put into getSuspectedColor for the pure panel-recognition color. */
public Color getSeenColor() {
  if (colorSensor.getColor() != null)
  {lastSeenColor =colorSensor.getColor(); }
  return lastSeenColor;
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
  if (getSuspectedColor(currentColor) != null)
    suspectedColor = getSuspectedColor(currentColor);

  }

  public Color getCurrentColor() {return currentColor;}

  PanelColors lastSeenPanelColor;
  /**
   * Converts from 'color' to 'PanelColor': assumes no overlap
   * 
   * @param colour the color to check
   * @return the determined color (may be null)
   */
  public PanelColors getSuspectedColor(Color colour) { // Mr. Wertz approved -CMM
   PanelColors p = lastSeenPanelColor;
    for (PanelColors isItMe : PanelColors.values()) {
      if (isItMe.withinTolerance(colour.red, colour.green, colour.blue)) {
        lastSeenPanelColor = p = isItMe;
      }
    }
    if (p == null && lastSeenPanelColor != null)
    {p = lastSeenPanelColor;}
    if (p == null && lastSeenPanelColor == null)
    {p = PanelColors.nocolor;}
    return p;
  }
/** find the last updated value for the suspected colors. */
  public PanelColors getSuspectedColor() {return suspectedColor;}

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
    //TODO: ONLY FOR TESTING, DO NOT KEEP IN FINAL CODE 
    gameData = "B";
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
        return PanelColors.nocolor; // corrupt data! uh oh!
      }
    } else {
      receivedGameColor = false;
      return PanelColors.nocolor;// we haven't gotten it yet
    }
  }

  /**
   * Returns the approximate distance from the color scanner (At a high value,
   * color may not be accurate)
   * 
   * @return distance from color scanner from 0-2047. In practice, it only goes from about 95 anywhere over 8 1/2 in, exponentially increasing to 2047 as it gets closer.
   */
  public int getProximity() {
    return colorSensor.getProximity();// approximate effective range: 8 and 1/2 inches
  }

  /** given the color under the wheel now and the color we want to be under the wheel, calculate the minimum number of encoder ticks 
   * (and the direction, hence positive or negative values) the motor must turn to get to the color we want to be under the sensor.
   * @param colorNow the color under the sensor, as a starting point
   * @param colorWant the color we want to get to be under the sensor
   *  */
  public double getPathToDesiredColor(PanelColors colorNow, PanelColors colorWant) {
    double retVal;
    /* 4 colors, repeated; each slice takes up 1/8 of the wheel. 
    if clockwise is the positive direction, the number of slices needed
     to turn to get to the desired color is as follows.

    colorNow | colorWanted
             | R | G | B | Y |
          R  | 0 |-1 |+-2|+1 |
          G  |+1 | 0 |-1 |+-2|
          B  |+-2|+1 | 0 |-1 |
          Y  |-1 |+-2|+1 | 0 |
    */
int slicesVal;
if (colorNow != null) {
switch (colorNow) {
          case red:
          switch (colorWant) {
            case red:    slicesVal = 0; break;
            case green:  slicesVal = -1; break;
            case blue:   slicesVal = 2; break;
            case yellow: slicesVal = 1; break;
            default:     slicesVal = 0; break;
          }
          break;

          case green:
          switch (colorWant) {
            case red: slicesVal = 1; break;
            case green: slicesVal = 0; break;
            case blue: slicesVal = -1; break;
            case yellow: slicesVal = 2; break;
            default:   slicesVal = 0;  break;
            }
          break;

          case blue:
          switch (colorWant) {
            case red:  slicesVal = 2;  break; 
            case green: slicesVal = 1;  break;
            case blue:  slicesVal = 0;  break;
            case yellow:   slicesVal = -1;  break;
            default:   slicesVal = 0; break;
            }
            break;
            case yellow:
            switch (colorWant) {
            case red:  slicesVal = -1;  break;
            case green:  slicesVal = 2; break;
            case blue:  slicesVal = 1;  break;
            case yellow:  slicesVal = 0;  break;
            default:   slicesVal = 0;   break;
          }
break;
default:
slicesVal = 0;
break;
}//switch
}//if colorNow != null
else
{slicesVal = 0;}

//slicesVal to revolutions to encoder ticks: 1/8 rev per slicesVal: slicesVal/1 * ( 1/8 rev /1 slicesVal unit) = rev
retVal = controlPanelTargetRevolutionsToQuadEncoderTicks(RobotMap.controlPanelDirectionFactor *slicesVal/8.);
    return retVal;
  }

  public enum PanelColors {
    blue(0.13, 0.44, 0.43, "blue"), green(0.17, 0.59, 0.25, "green"), red(0.5, 0.36, 0.14, "red"), yellow(0.32, 0.57, 0.12, "yellow"), nocolor(0,0,0,"no color");
    // TODO: Find more accurate values, also reformat
    // Maybe use HSV for tolerance check?

    // numbers gotten from game manual + online converter
    final private double redVal, greenVal, blueVal;

    final private String name;
    PanelColors(double r, double g, double b, String n) {
      redVal = r;
      greenVal = g;
      blueVal = b;
      name = n;
    }

    public Color getColor() {
      Color c = new Color(redVal, greenVal, blueVal);
      return c;
    }
    public String getName() {return name;}

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