// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class IntakeConstants {
    public static final int kintakeMotor1Id = 24;
    public static final int kintakeMotor2Id = 25; //TODO: change it lol 
    public static final int kindexerMotorId = 7; //TODO: change it lol
  }
  public static class SpeakerShooterConstants {
    public static final int kspeakerShooterMotorId = 26;
    public static final int kspeakerShooterMotorId2 = 27;
  }
  public static class AmpShooterConstants {
        public static final int kampShooterMotorId = 6; //TODO: change it lol
    public static final int kampAxisMotor1Id = 4; //TODO: change it lol
    public static final int kampAxisMotor2Id = 5; //TODO: change it lol
  }
}
