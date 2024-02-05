package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.lib.math.Transformations;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignCommand extends Command {
    private final SwerveSubsystem swerve;

    public AutoAlignCommand(SwerveSubsystem swerveSubsystem) {
        swerve = swerveSubsystem;
    }

    private static final Transform3d robotToShooter = new Transform3d();
    private static final Pose3d speaker = new Pose3d();
    private static final double maximumFiringAngle = 75 * Math.PI / 180;

    private Pose2d calculateDestination() {
        var currentPose = new Pose3d(swerve.getPose());
        var speakerToRobot = currentPose.minus(speaker);
        
        // restrict the angle to a -180 to 180 degree range
        double angle = speakerToRobot.getRotation().getZ();
        while (Math.abs(angle) > Math.PI) {
            angle -= Math.PI * 2 * Math.signum(angle);
        }

        // make sure were moving to a valid firing position
        angle = MathUtil.clamp(angle, -maximumFiringAngle, maximumFiringAngle);

        // calculate robot (center) distance to speaker
        var speakerToShooter = speakerToRobot.plus(robotToShooter);
        double height = Math.abs(speakerToShooter.getZ());
        double shooterAngle = Math.abs(robotToShooter.getRotation().getX());
        double firingDistance = height / Math.tan(shooterAngle) - robotToShooter.getY(); // see whiteboard

        // calculate the actual desired position of the robot
        double directionRotation = angle + speaker.getRotation().getZ();
        var direction = new Translation2d(Math.cos(directionRotation), Math.sin(directionRotation));
        var position = speaker.getTranslation().toTranslation2d().plus(direction.times(firingDistance));

        return new Pose2d(position, new Rotation2d(-directionRotation));
    }

    @Override
    public void execute() {
        var destination = calculateDestination();
        var dtpCommand = new DriveToPosition(swerve, swerve::getPose, destination);

        // Run the drive command
        dtpCommand.schedule();
        new PrintCommand("[Auto Align]: It works").schedule();
    }
}
