package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ControlPanelSubsystem extends Subsystem {
  // TODO: Find stuff for JE-PLG-149 motors so they can be used here

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  private WPI_TalonSRX diskSpinnerTalon;// = new WPI_TalonSRX(RobotMap.diskSpinnerMotorID);
  private DoubleSolenoid diskSpinnerSolenoid;// = new DoubleSolenoid(RobotMap.ColorWheelSolenoidForwardChannel,RobotMap.ColorWheelSolenoidReverseChannel);

  private Color targetColor;
  private boolean receivedGameColor = false;
  private boolean colorsAligned = false;
  private Color currentColor;

  final Color blueColor = new Color(0, 1, 1);// number gotten from game manual + online converter
  final Color greenColor = new Color(0, 1, 0);
  final Color redColor = new Color(1, 0, 0);
  final Color yellowColor = new Color(1, 0, 0);

  final double toleranceSize = .01;// tolerance size (in percent)
  private boolean wasAligned = false;
  private double spinSpeed = 0.5;

  public void putSeenColor(){
    updateColorState();
    SmartDashboard.putNumber("Spotted Color: Red", currentColor.red*255);
    SmartDashboard.putNumber("Spotted Color: Green", currentColor.green*255);
    SmartDashboard.putNumber("Spotted Color: Blue", currentColor.blue*255);
  }

  //NOTE:  Will be implementing Rotation Control as a command
  public void spinToTargetColor(){
    if(checkColorAlignment()){
      diskSpinnerTalon.setNeutralMode(NeutralMode.Brake);
      diskSpinnerTalon.set(0);
      //TODO: Use actual motion profiling (note: maybe not, will definitely be very hard);
    }
    else{ 
      diskSpinnerTalon.set(spinSpeed);
    }

    if(wasAligned != colorsAligned){//if alignment has changed since last check...
      spinSpeed *= -.8; //Switch spin directions and decrease speed: we overshot
    }
    wasAligned = colorsAligned;
  }
/**Sets the value of this subsystem's DoubleSolenoid: kForward, kReverse, or kOff.  These are values of the enum DoubleSolenoid.Value. */
 public void setSolenoid(DoubleSolenoid.Value val) {
    diskSpinnerSolenoid.set(val);
    
 }
  
  /**
   * Update the color that we should be reading:
   */
  public void updateColorState() {
    targetColor = getGameTargetColor();
    currentColor = colorSensor.getColor();
  }

  /**
   * Checks whether the colors are aligned: null checked
   * @return true if aligned: false otherwise
   */
  public boolean checkColorAlignment(){
    updateColorState();

    //Quick null check:
    if(targetColor == null || currentColor == null){return colorsAligned = false;}

    if(//NOTE: Copy-paste is bad practice.  I am being a bad boy. -CMM
      Math.abs(targetColor.red - currentColor.red) < toleranceSize &&
      Math.abs(targetColor.green - currentColor.green) < toleranceSize &&
      Math.abs(targetColor.blue - currentColor.blue) < toleranceSize)
      {
        return colorsAligned = true;
    }
    return colorsAligned = false;
  }

  /**
   * Get what the game would like our sensor to read
   * 
   * @return color that our sensor should read
   */
  public Color getGameTargetColor() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      receivedGameColor = true;
      switch (gameData.charAt(0)) {
      case 'B':
        return redColor;
      case 'G':
        return yellowColor;
      case 'R':
        return blueColor;
      case 'Y':
        return greenColor;
      default:
        return null; // corrupt data! uh oh!
      }
    } else {
      receivedGameColor = false;
      return null;// we haven't gotten it yet
    }
  }

  @Override
  public void initDefaultCommand() {

  }

}