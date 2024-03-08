package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Transformations;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignCommand {
    private final SwerveSubsystem swerve;

    public AutoAlignCommand(SwerveSubsystem swerveSubsystem) {
        swerve = swerveSubsystem;
    }

    private static final Transform3d robotToShooter = new Transform3d(0, 0, 0.5207, new Rotation3d(0, 37.5 * Math.PI / 180, Math.PI));
    private static final Pose3d redSpeaker = new Pose3d(16.427, 5.548, 2.032, new Rotation3d(0, 0, Math.PI));
    private static final Pose3d blueSpeaker = new Pose3d(0.073, 5.548, 2.032, new Rotation3d(0, 0, 0));
    private static final double maximumFiringAngle = 75 * Math.PI / 180;

    private Pose2d calculateDestination() {
        var alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Red;
        var speaker = isBlue ? redSpeaker : blueSpeaker;

        Pose3d robot = new Pose3d(swerve.getPose());
        Pose3d shooter = robot.plus(robotToShooter);
        Transform3d speakerToRobot = robot.minus(speaker);

        // restrict the angle to a -180 to 180 degree range
        Translation3d speakerToRobotTranslation = speakerToRobot.getTranslation();
        double targetAngle = Math.atan2(speakerToRobotTranslation.getY(), speakerToRobotTranslation.getX());

        while (Math.abs(targetAngle) > Math.PI) {
            targetAngle -= Math.PI * 2 * Math.signum(targetAngle);
        }

        // make sure were moving to a valid firing position
        targetAngle = MathUtil.clamp(targetAngle, -maximumFiringAngle, maximumFiringAngle);

        // get pitch of shooter
        Translation3d shooterDirection = new Translation3d(1, shooter.getRotation());
        Translation2d shooterDirectionHorizontal = Transformations.normalize(shooterDirection.toTranslation2d());
        double cosPitch = Transformations.dot(shooterDirection.toTranslation2d(), shooterDirectionHorizontal);
        double pitch = Math.acos(cosPitch);
        double cotPitch = cosPitch / Math.sin(pitch);

        // calculate robot (center) distance to speaker
        Transform3d speakerToShooter = speakerToRobot.plus(robotToShooter);
        double height = Math.abs(speakerToShooter.getZ());
        double shooterFiringDistance = height * cotPitch;

        // calculate the actual desired position of the robot
        Rotation2d directionRotation = new Rotation2d(targetAngle).plus(speaker.getRotation().toRotation2d());
        Translation2d desiredRelativeTranslation = new Translation2d(shooterFiringDistance, directionRotation);

        Translation2d desiredPosition = speaker.getTranslation().toTranslation2d().plus(desiredRelativeTranslation);
        Rotation2d desiredRotation = robotToShooter.getRotation().toRotation2d().minus(directionRotation);

        return new Pose2d(desiredPosition, desiredRotation);
    }

    public Command getCommand() {
        Pose2d destination = calculateDestination();
        return new DriveToPositionPathPlanner(swerve.getPose(), destination).getCommand();
    }
}