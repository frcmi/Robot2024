// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// code now takes in everything color related except for just RGB

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
public class DriveStationSubsystem extends SubsystemBase {
  public Color8Bit setColor = new Color8Bit(128,0,0);


  public AddressableLEDBuffer m_ledBuffer;


  public void setLight(int ID, Color8Bit color){
    m_ledBuffer.setLED(ID, color);
  }

  public AddressableLED m_led;
  public DriveStationSubsystem() {
    m_led = new AddressableLED(LEDConstants.kLedPort);
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLedCount);

    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }
 
  // TODO: See if still needed, or remove if isn't.
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
    return runOnce(
        () -> {
          rainbow();
          m_led.setData(m_ledBuffer);
        });
  }

  public void setColor(Color8Bit color) {

    setColor = color;

  }

  public Command setLights(){
    return runOnce(() -> {
      System.out.println("bombaclat");
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, setColor);
      }
    });
  }



  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  @Override
  public void periodic() {
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
  public void dropDisk(){
    setColor(new Color8Bit(255,165,0));
  }
  public void coop(){
    setColor(new Color8Bit(255,255,0));
  }
  public void ampSpeaker(){
    setColor(new Color8Bit(128,0,128));
  }
  public void readyToAmp(){
    setColor(new Color8Bit(0, 0, 255)); //change color later bc alliance problems
  }
  public void readyToSpeaker(){
    setColor(new Color8Bit(0, 255, 0));
  }
}
