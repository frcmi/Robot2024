// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// code now takes in everything color related except for just RGB

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color8Bit;
public class DriveStationSubsystem extends SubsystemBase {
  public Color8Bit setColor = new Color8Bit(128,0,0);


  public AddressableLEDBuffer m_ledBuffer;
  public AddressableLEDBuffer m_ledBuffer2;
  private SwerveSubsystem swerve;

<<<<<<< Updated upstream
  // public DigitalInput beambreak = new DigitalInput(0);
=======
  public DigitalInput beambreak = new DigitalInput(0);
>>>>>>> Stashed changes

  public void setLight(int ID, Color8Bit color){
    m_ledBuffer.setLED(ID, color);
    
  }

  public AddressableLED m_led;

  public DriveStationSubsystem( SwerveSubsystem swerveSubsystem) {
    m_led = new AddressableLED(LEDConstants.kLedPort);
    m_ledBuffer = new AddressableLEDBuffer(73);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    this.swerve = swerveSubsystem;


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

  public Command setColor(Color8Bit color) {
    return runOnce(() -> {
      setColor = color;
    }).andThen(setLights());
  
  }
  public Command ledOff() {  
    return runOnce(
      () -> {
        setColor(new Color8Bit(0,0,0));
      }
    );
  }
  public Command setLights(){
    return runOnce(() -> {
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

    Pose2d pose = swerve.getPose();

<<<<<<< Updated upstream
    // if (beambreak.get()) {
    //   readyToSpeaker().schedule();
    // }
=======
    if (beambreak.get()) {
      readyToSpeaker().schedule();
    }
>>>>>>> Stashed changes
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
  public Command dropDisk(){
    return setColor(new Color8Bit(0,255,100));
  
  }
  public Command coop(){
     return setColor(new Color8Bit(255,255,0));
    
  }
  public Command ampSpeaker(){
    return setColor(new Color8Bit(128,0,128));
    
  }
  public Command readyToAmp(){
     return setColor(new Color8Bit(255, 192, 203));
     //change color later bc alliance problems
  }
  public Command readyToSpeaker(){
    return setColor(new Color8Bit(0, 255, 0));
    
  }
}
