// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
public class DriveStationSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  

  public AddressableLEDBuffer m_ledBuffer;

  public AddressableLED m_led;
  public DriveStationSubsystem() {
    m_led = new AddressableLED(OperatorConstants.kDriverControllerPort);
    m_ledBuffer = new AddressableLEDBuffer(OperatorConstants.kLedCount);

    // Default to a length of 60, start empty output

    // Length is expensive to set, so only set it once, then just update data

    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
  //  setColor(255, 0, 0);
  //  m_led.setData(m_ledBuffer);
  //  m_led.start();

  }

  private void setColor(Color8Bit color) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setLED(i, color);
   }
  }

  private void rainbow() {

    // For every pixel

    int m_rainbowFirstPixelHue = 0;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      // Calculate the hue - hue is easier for rainbows because the color

      // shape is a circle so only one value needs to precess

      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;

      // Set the value

      m_ledBuffer.setHSV(i, hue, 255, 128);

    }

    // Increase by to make the rainbow "move"

    m_rainbowFirstPixelHue += 3;

    // Check bounds

    m_rainbowFirstPixelHue %= 180;

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command runRainbow() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          rainbow();
          m_led.setData(m_ledBuffer);
        });
  }

  public Command run(Color8Bit color) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setColor(color);
          m_led.setData(m_ledBuffer);
          m_led.start();
          //System.out.println("CODE IS WORKING!! ");
        });
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}