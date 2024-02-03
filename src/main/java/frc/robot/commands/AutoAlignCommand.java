package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignCommand extends Command
{
    public static final Translation2d TARGET_POSITION = new Translation2d(15, 22);
    public static final float MAXIMUM_ANGLE_DIFFERENCE = 30f;
    public static final float DESIRED_FIRING_DISTANCE = 10f;

    private final VisionSubsystem visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    private final Supplier<Pose2d> currentPosition;
    Pose2d destination;

    public AutoAlignCommand(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem, Supplier<Pose2d> currentPosition)
    {
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.currentPosition = currentPosition;
    }

    private void calculateDestination()
    {
        // todo: ask hadrian
    }

    @Override
    public void execute()
    {
        DriveToPosition dtpCommand = new DriveToPosition(this.swerveSubsystem, currentPosition, destination);
        
        // Run the drive command
        dtpCommand.schedule();
        new PrintCommand("[Auto Align]: It works").schedule();
    }
}
