package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

public class ControlPanelSubsystem extends Subsystem {
  // TODO: Find stuff for JE-PLG-149 motors so they can be used here

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  /**
   * Returns the Color object of the color sensor
   * 
   * @return Current detected color object
   */
  public Color getColor() {
    return colorSensor.getColor();
  }

  /**
   * Returns the value for the raw infrared light detected as a double
   * 
   * @return Raw infrared light value
   */
  public double getInfrared() {
    return colorSensor.getIR();
  }

  /**
   * Returns the red value of the RGB for the color sensor as a double
   * 
   * @return Red value as double
   */
  public double getRed() {
    return colorSensor.getColor().red;
  }

  /**
   * Returns the green value of the RGB for the color sensor as a double
   * 
   * @return Green value as double
   */
  public double getGreen() {
    return colorSensor.getColor().green;
  }

  /**
   * Returns the blue value of the RGB for the color sensor as a double
   * 
   * @return Blue value as double
   */
  public double getBlue() {
    return colorSensor.getColor().blue;
  }

  /**
   * Returns the RGB values of the detected color for the color sensor as doubles
   * in the order of {Red,Green,Blue}
   * 
   * @return {R,G,B} values as a double array
   */
  public double[] getRGB() {
    double[] temporaryArray = { colorSensor.getColor().red, colorSensor.getColor().green, colorSensor.getColor().blue };
    return temporaryArray;
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

  @Override
  public void initDefaultCommand() {

  }

}