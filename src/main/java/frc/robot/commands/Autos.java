// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command testAuto(SwerveSubsystem drive, IntakeSubsystem intake, Supplier<Pose2d> currentPosition) {
    DriveToPositionPathPlanner path = new DriveToPositionPathPlanner(drive, currentPosition, new Pose2d(new Translation2d(0, 2), new Rotation2d(0)));
    DriveToPositionPathPlanner path2 = new DriveToPositionPathPlanner(drive, currentPosition, new Pose2d(new Translation2d(0, 6), new Rotation2d(0)));
    return path.andThen(intake.intakeSpeaker()).withTimeout(3)
    .andThen(intake.stop()).withTimeout(3)
    .andThen(path2.andThen(intake.intakeSpeaker()).withTimeout(3))
    .andThen(intake.stop().withTimeout(3));
    // return new DriveToPositionPathPlanner(drive, currentPosition, new Pose2d(new Translation2d(2, 2), new Rotation2d(0))).withTimeout(2)
    // .andThen(new DriveToPosition(drive, currentPosition, new Pose2d(new Translation2d(0, -2), new Rotation2d(Math.toRadians(180)))));
  }
}
