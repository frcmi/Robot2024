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
  public static class SwerveConstants { //TODO: asactually fill these in (they are from kickoff bot)
    public static class AutoConstants {
      public static final double kSteerP = 0.5;
      public static final double kSteerI = 0;
      public static final double kSteerD = 0;

      public static final double kRotationP = 0.5;
      public static final double kRotationI = 0;
      public static final double kRotationD = 0;

      public static final double kAutoXP = 0;
      public static final double kAutoXI = 0;
      public static final double kAutoXD = 0;

      public static final double kAutoYP = 0;
      public static final double kAutoYI = 0;
      public static final double kAutoYD = 0;
    }
    // limits
    public static final double kAllowedDistanceToDestination = 0.1;
    public static final double kMaxSpeed = 0.1;
    public static final double kMaxAcceleration = 0.05;

    // gear ratios
    public static final double kSteerMotorGearRatio = 150d/7d;
    public static final double kDriveMotorGearRatio = 6.12d;
    public static final double kCouplingGearRatio = 0;

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
    public static final double kDriveBaseRadius = 0.5028;
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
      .withCouplingGearRatio(kCouplingGearRatio)
      .withSteerMotorInverted(false);

    public static final SwerveModule Mod0 = new SwerveModule(
      0, 
      Rotation2d.fromRadians(-3.063359633407626),
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
      Rotation2d.fromRadians(-0.484737928971863),
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
      Rotation2d.fromRadians(3.115514980195737), 
      kSwerveConstantsFactory.createModuleConstants(
        7, 
        3, 
        11, 
        0, 
        -kWidth/2, 
        -kLength/2, 
        false));
    public static final SwerveModule Mod3 = new SwerveModule(
      3, 
      Rotation2d.fromRadians(-2.437495471950284), 
      kSwerveConstantsFactory.createModuleConstants(
        8, 
        4, 
        12, 
        0, 
        kWidth/2, 
        -kLength/2, 
        false));
  }
}
