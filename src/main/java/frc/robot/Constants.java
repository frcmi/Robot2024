// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Rotation2d;

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
  public static class SwerveConstants { //TODO: actually fill these in (they are from kickoff bot)
    // limits
    public static final double kAllowedDistanceToDestination = 0.1;
    public static final double kMaxSpeed = 3;
    public static final double kMaxAcceleration = 3;

    // gear ratios
    public static final double kSteerMotorGearRatio = 150d/7d;
    public static final double kDriveMotorGearRatio = 6.12d;

    // ids
    public static final int kPigeonId = 0;
      // RIP Caine Busse, 1/11/2024 3:48PM - 1/11/2024 3:52PM
    public static final String kCanbusName = "rio";
    
    // interpolation
    public static final double kDriveS = 0;
    public static final double kDriveV = 0;
    public static final double kDriveA = 0;

    public static final Slot0Configs kSteerMotorGains = new Slot0Configs();
    public static final Slot0Configs kDriveMotorGains = new Slot0Configs();
    public static final SteerFeedbackType kFeedbackSource = SteerFeedbackType.RemoteCANcoder;

    // measured values
    public static final double kWidth = 0.7112;
    public static final double kLength = 0.7112;
    public static final double kWheelRadius = 2;
    public static final double kSteerIntertia = 50;
    public static final double kDriveIntertia = 50;

    // electricity
    public static final double kSlipCurrent = 20;
    public static final double kSpeed12Volts = 4;
    
    private static final SwerveModuleConstantsFactory kSwerveConstantsFactory = new SwerveModuleConstantsFactory()
      .withDriveMotorGearRatio(kDriveMotorGearRatio)
      .withSteerMotorGearRatio(kSteerMotorGearRatio)
      .withWheelRadius(kWheelRadius)
      .withSlipCurrent(kSlipCurrent)
      .withSteerMotorGains(kSteerMotorGains)
      .withDriveMotorGains(kDriveMotorGains)
      .withSpeedAt12VoltsMps(kSpeed12Volts)
      .withSteerInertia(kSteerIntertia)
      .withDriveInertia(kDriveIntertia)
      .withFeedbackSource(kFeedbackSource)
      .withSteerMotorInverted(false);

    public static final SwerveModule Mod0 = new SwerveModule(
      0, 
      Rotation2d.fromDegrees(329.59), 
      kSwerveConstantsFactory.createModuleConstants(
        6, 
        2, 
        10, 
        0, 
        kWidth/2, 
        kLength/2, 
        false));
    public static final SwerveModule Mod1 = new SwerveModule(
      1, 
      Rotation2d.fromDegrees(136.93), 
      kSwerveConstantsFactory.createModuleConstants(
        5, 
        1, 
        9, 
        0, 
        -kWidth/2, 
        kLength/2,
        false));
    public static final SwerveModule Mod2 = new SwerveModule(
      2, 
      Rotation2d.fromDegrees(126.83), 
      kSwerveConstantsFactory.createModuleConstants(
        3, 
        7, 
        11, 
        0, 
        -kWidth/2, 
        -kLength/2, 
        false));
    public static final SwerveModule Mod3 = new SwerveModule(
      3, 
      Rotation2d.fromDegrees(110.65), 
      kSwerveConstantsFactory.createModuleConstants(
        4, 
        8, 
        12, 
        0, 
        kWidth/2, 
        -kLength/2, 
        false));
  }
}
