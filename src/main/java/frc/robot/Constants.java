// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.COTSTalonFXSwerveConstants;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

import java.util.concurrent.TimeUnit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class TelemetryConstants {
        // DON'T ENABLE UNLESS ABSOLUTELY NEEDED
        // this will fully disable logging even when FMS is connected.
        public static final boolean killswitch = false;
        // If true data won't be sent over network even when not connected to FMS
        public static final boolean disableNetworkLogging = false;
        // ONLY ENABLE IN DEV (this *should* be overwritten when connected to FMS, but that's untested)
        public static final boolean disableDatalog = true;
        // Prefix in NetworkTables, must end with a '/'
        public static final String tabPrefix = "UltraLog/";
        // How often to re-check if the FMS is connected (and disable network logging if so)
        public static final double fmsCheckDelay = TimeUnit.SECONDS.toMillis(1);
    }

    public static class LEDConstants {
      public static final int kStreakLength = 3; //TODO: Change Streak Length
      public static final int kLedCount = 21;
    
      public static final int kLedPort = 1;
    }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverButtonPort = 1;
    public static final double stickDeadband = 0.03;
  }

  public static final class SwerveConstants {
    public static final double kAllowedDistanceToDestination = 0.1;
    public static final double kAllowedRotationDifferenceToDestination = 0.1; // Radians

    public static final int pigeonID = 0;

    public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(28);
    public static final double wheelBase = Units.inchesToMeters(28);
    public static final double wheelCircumference = chosenModule.wheelCircumference;
    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    public static final float currentLimitModifier = 0.4f;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = (int)(25 * currentLimitModifier);
    public static final int angleCurrentThreshold = (int)(40 * currentLimitModifier);
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = (int)(35 * currentLimitModifier);
    public static final int driveCurrentThreshold = (int)(60 * currentLimitModifier);
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    // TODO: REMOVE
    public static final double kOdometryProportionalityConstant = 1;
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /** Volts (out of 12) */
    public static final double maxSpeed = 12; // TODO: This must be tuned to specific robot
    /** Volts (out of 12) */
    public static final double maxAngularVelocity = 12; // TODO: This must be tuned to specific robot

    /* Sensitivity Values */
    public static final double translationSensitivity = 0.75;
    public static final double rotationSensitivity = 0.5;

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 9; //
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(-2.317844970495204);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final boolean isInverted = true;
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(-2.839398438376322);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final boolean isInverted = true;
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(-2.83633047680055);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final boolean isInverted = true;
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(-2.481980914798967);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final boolean isInverted = true;
    }
  }

  public static class SimulationConstants {
    public static final double kSimulationMaxSpeed = 2;
    public static final double kSimulationMaxRotationSpeed = 3;
  }

  public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
                                            // to specific robot
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        
    public static final double kRotationP = 10;
    public static final double kRotationI = 0;
    public static final double kRotationD = 0.05;
    // For PathPlanner
    public static final double kAccelerationP = 25;
    public static final double kAccelerationI = 0;
    public static final double kAccelerationD = 0.01;
    // need to be tuned, a lot
  }
  public static class IntakeConstants {
    public static final int kIntakeMotor1Id = 14;
    public static final int kIntakeMotor2Id = 25;
    public static final int kIndexerMotorId = 26;

    public static final int kSpeakerBeamBreakPort = 2;
    public static final double kSpeakerShootSpeed = 2;

    public static final double kIntakeMotorSpeed = 0.2085;
    public static final double kIndexerSpeed = 0.2085;
  }

  public static class SpeakerShooterConstants {
    public static final int kSpeakerShooterMotorId = 27;

    public static final double kSpeakerMotorSpeed = -0.85;
  }

  public static class AmpShooterConstants {
    public static final double kAmpMotorSpeed = 0.83;
    public static int kShootMotor = 31;
    public static double kIntakeCurrentLimitAmps = 100000000;

    public static final int kAmpBeamBrakeId = 4;

  }

  public static class AmpArmConstants {
    public static final double kMaxArmVolts = 6;
    public static final int kAmpArmMotorId = 32;
    public static final int kArmEncoderId = 0; // TODO: Change ID based on DIO port
    public static final double kTorqueArmConstant = 0.6;
    public static final double kGravityLimit = 0.3;
    // Gareths law of constants
    // TODO: Tune all of these values
    // PID parameters
    public static final double kP = 4.3;
    public static final double kI = 0;
    public static final double kD = 0.1;

    // FeedForward parameters
    public static final double kS = 0.0;
    public static final double kG = 0.16;
    public static final double kV = 3.5;
    public static final double kA = 0.01;

    // Motion Profile
    public static final double kMaxArmVel = 0.5;
    public static final double kMaxArmAccel = 3;

    // Angles
    public static final double kMaxAngle = 180; // TODO: Maximum angle of arm
    public static final double kMinAngle = 0; // TODO: Change to resting position
    public static final double kShootAngle = 92;

    public static final double kAmpEncoderOffset = 179 + 140 + 4.2 - 124 + 1.5; // Needs to be measured

    // Raise/Lower Constants
    public static final double kRaiseArmVolts = 1;
    public static final double kLowerArmVolts = -1;
    public static final double kAmpCurrentLimit = 39;
  }

  public class ClimberConstants {
    public static final int kRightClimberId = 50;
    public static final int kLeftClimberId = 51;

    public static final double kClimberUp = 0.3;
    public static final double kClimberDown = -0.7;
  }

  public static class VisionConstants {
      public static final Transform3d robotToCamera = new Transform3d(Units.inchesToMeters(5), Units.inchesToMeters(9), Units.inchesToMeters(21.85), new Rotation3d(0, -Math.toRadians(10.5),Math.PI));
  }

  public static class AutoAlignConstants {
    public static final Transform3d kRobotToShooter = new Transform3d(0, 0, 0.5207, new Rotation3d(0, 77.5 * Math.PI / 180, Math.PI));
    public static final Pose3d kRedSpeaker = new Pose3d(16.427, 5.548, 2.032, new Rotation3d(0, 0, Math.PI));
    public static final Pose3d kBlueSpeaker = new Pose3d(0.073, 5.548, 2.032, new Rotation3d(0, 0, 0));
    public static final double kMaximumFiringAngle = 75 * Math.PI / 180;
  }
}