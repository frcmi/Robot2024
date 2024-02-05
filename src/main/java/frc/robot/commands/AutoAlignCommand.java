package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignCommand extends Command {
    private final SwerveSubsystem swerve;

    public AutoAlignCommand(SwerveSubsystem swerveSubsystem) {
        swerve = swerveSubsystem;
    }

    private static final Pose2d speaker = new Pose2d();
    private static final double maximumFiringAngle = 75 * Math.PI / 180;

    private Pose2d calculateDestination() {
        var currentPose = swerve.getPose();
        var speakerToRobot = currentPose.minus(speaker);
        
        // restrict the angle to a -180 to 180 degree range
        double angle = speakerToRobot.getRotation().getRadians();
        while (Math.abs(angle) > Math.PI) {
            angle -= Math.PI * 2 * Math.signum(angle);
        }

        // make sure were moving to a valid firing position
        angle = MathUtil.clamp(angle, -maximumFiringAngle, maximumFiringAngle);

        double directionRotation = angle + speaker.getRotation().getRadians();
        double firingDistance = 2; // todo(nora?): calculate (measurements should be on robotics whiteboard)

        var direction = new Translation2d(Math.cos(directionRotation), Math.sin(directionRotation));
        var position = speaker.getTranslation().plus(direction.times(firingDistance));

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
