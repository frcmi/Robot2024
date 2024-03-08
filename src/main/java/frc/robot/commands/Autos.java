// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
    Command path = driveToPosition(new Pose2d(new Translation2d(0, 2), new Rotation2d(0)));

    return intake.intakeSpeaker().withTimeout(0.5);
    // .andThen(intake.stop()).withTimeout(1)
    // .andThen(path.alongWith(intake.intakeSpeaker())).withTimeout(5)
    // .andThen(intake.intakeSpeaker());

    // return new DriveToPositionPathPlanner(drive, currentPosition, new Pose2d(new
    // Translation2d(2, 2), new Rotation2d(0))).withTimeout(2)
    // .andThen(new DriveToPosition(drive, currentPosition, new Pose2d(new
    // Translation2d(0, -2), new Rotation2d(Math.toRadians(180)))));
  }

  public static Command ppAuto(SwerveSubsystem drive) {
    return Commands.runOnce(() -> drive.setPose(PathPlannerAuto.getStaringPoseFromAutoFile("RE-ADD NAME")), drive)
        .andThen(new PathPlannerAuto("RE-ADD NAME"));
    // .andThen(new PathPlannerAuto("Pickup Note 4 Auto")));
    // now goes from another starting postition to its actual starting position
    // .andThen(shooter.shootSpeaker()).withTimeout(2);
  }

  public static Command pathplannerPath(String path) {
    return new PathPlannerAuto(path);
  }

  public static Command driveToPosition(Pose2d pose) {
    System.out.println("Moving to <" + pose.getX() + "," + pose.getY() + ">");
    PathConstraints constraints = new PathConstraints(
        1.5, 2.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
  }

  public static Command autoShoot() {
    return null;
  }
}
