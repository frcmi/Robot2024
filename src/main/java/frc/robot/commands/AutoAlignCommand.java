package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Transformations;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignCommand extends Command {
    private final SwerveSubsystem swerve;
    private Command driveToPosition;

    public AutoAlignCommand(SwerveSubsystem swerveSubsystem) {
        swerve = swerveSubsystem;
        driveToPosition = null;
    }

    private Pose2d calculateDestination() {
        var alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Red;
        var speaker = isBlue ? AutoAlignConstants.kRedSpeaker : AutoAlignConstants.kBlueSpeaker;

        Pose3d robot = new Pose3d(swerve.getPose());
        Pose3d shooter = robot.plus(AutoAlignConstants.kRobotToShooter);
        Transform3d speakerToRobot = robot.minus(speaker);

        // restrict the angle to a -180 to 180 degree range
        Translation3d speakerToRobotTranslation = speakerToRobot.getTranslation();
        double targetAngle = Math.atan2(speakerToRobotTranslation.getY(), speakerToRobotTranslation.getX());

        while (Math.abs(targetAngle) > Math.PI) {
            targetAngle -= Math.PI * 2 * Math.signum(targetAngle);
        }

        // make sure were moving to a valid firing position
        targetAngle = MathUtil.clamp(targetAngle, -AutoAlignConstants.kMaximumFiringAngle, AutoAlignConstants.kMaximumFiringAngle);

        // get pitch of shooter
        Translation3d shooterDirection = new Translation3d(1, shooter.getRotation());
        Translation2d shooterDirectionHorizontal = Transformations.normalize(shooterDirection.toTranslation2d());
        double cosPitch = Transformations.dot(shooterDirection.toTranslation2d(), shooterDirectionHorizontal);
        double pitch = Math.acos(cosPitch);
        double cotPitch = cosPitch / Math.sin(pitch);

        // calculate robot (center) distance to speaker
        Transform3d speakerToShooter = speakerToRobot.plus(AutoAlignConstants.kRobotToShooter);
        double height = Math.abs(speakerToShooter.getZ());
        double shooterFiringDistance = height * cotPitch;

        // calculate the actual desired position of the robot
        Rotation2d directionRotation = new Rotation2d(targetAngle).plus(speaker.getRotation().toRotation2d());
        Translation2d desiredRelativeTranslation = new Translation2d(shooterFiringDistance, directionRotation);

        Translation2d desiredPosition = speaker.getTranslation().toTranslation2d().plus(desiredRelativeTranslation);
        Rotation2d desiredRotation = AutoAlignConstants.kRobotToShooter.getRotation().toRotation2d().minus(directionRotation);

        return new Pose2d(desiredPosition, desiredRotation);
    }

    @Override
    public void execute() {
        if (driveToPosition == null) {
            var destination = calculateDestination();
            driveToPosition = Autos.driveToPosition(destination);
        }

        driveToPosition.execute();
    }

    @Override
    public boolean isFinished() {
        if (driveToPosition != null && driveToPosition.isFinished()) {
            driveToPosition = null;
        }

        return driveToPosition == null;
    }
}