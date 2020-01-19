package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

public class ControlPanelSubsystem extends Subsystem {
  // TODO: Find stuff for JE-PLG-149 motors so they can be used here

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  @Override
  public void initDefaultCommand() {

  }

}