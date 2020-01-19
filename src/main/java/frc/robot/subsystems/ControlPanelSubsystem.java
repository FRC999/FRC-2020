package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

public class ControlPanelSubsystem extends Subsystem {
  // TODO: Find stuff for JE-PLG-149 motors so they can be used here

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  private Color targetColor;
  private boolean receivedGameColor = false;
  private boolean colorsAligned = false;
  private Color currentColor;

  final Color blueColor = new Color(0, 1, 1);// number gotten from game manual + online converter
  final Color greenColor = new Color(0, 1, 0);
  final Color redColor = new Color(1, 0, 0);
  final Color yellowColor = new Color(1, 0, 0);

  final double toleranceSize = .01;// tolerance size (in percent)

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