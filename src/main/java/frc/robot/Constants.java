// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class LEDConstants {
    public static final Color8Bit kYellow = new Color8Bit(255, 255, 0);
    public static final Color8Bit kPurple = new Color8Bit(255, 0, 255);
    public static final Color8Bit kInitialMaroon = new Color8Bit(144, 56, 32);
    public static final int kLightsPerFoot = 9; //also place holder
    public static final int[] kLightPorts = {0, 1}; // <== Placeholder!!
    public static final int[] kLightsLengthsArray = {kLightsPerFoot, kLightsPerFoot};
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  
  }
}


