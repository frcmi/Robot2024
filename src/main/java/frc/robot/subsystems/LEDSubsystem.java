package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.Color;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */

  protected Color8Bit currentColor;
  protected AddressableLED[] addressableLEDs;
  protected AddressableLEDBuffer[] addressableLEDBuffers;
  protected boolean increasing = true;
  protected boolean breathing;

public Command setColor(Color8Bit color) {
    return runOnce(() -> {
      for (int j = 0; j < addressableLEDs.length; j++) {
        for (int i = 0; i < addressableLEDBuffers[j].getLength(); i++) {
          addressableLEDBuffers[j].setLED(i, color);
        }
      }
    });
  }
}