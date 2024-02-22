// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpeakerShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command testAuto(SwerveSubsystem drive, IntakeSubsystem intake, Supplier<Pose2d> currentPosition) {
    DriveToPositionPathPlanner path = new DriveToPositionPathPlanner(drive, currentPosition, new Pose2d(new Translation2d(0, 2), new Rotation2d(0)));

    return intake.intakeSpeaker().withTimeout(0.5);
    // .andThen(intake.stop()).withTimeout(1)
    // .andThen(path.alongWith(intake.intakeSpeaker())).withTimeout(5)
    // .andThen(intake.intakeSpeaker());

    // return new DriveToPositionPathPlanner(drive, currentPosition, new Pose2d(new Translation2d(2, 2), new Rotation2d(0))).withTimeout(2)
    // .andThen(intake.intakeSpeaker())
    // .andThen(new DriveToPosition(drive, currentPosition, new Pose2d(new Translation2d(0, -2), new Rotation2d(Math.toRadians(180)))));
  }

  public static Command ppAuto(IntakeSubsystem intake, SpeakerShooterSubsystem shooter) {
    return ((new PathPlannerAuto("Example Auto")).withTimeout(5)
      .alongWith(intake.intakeSpeaker()));
      // .andThen(shooter.shootSpeaker()).withTimeout(2);
  }
}
