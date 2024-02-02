package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoAlignCommand;

public class VisionSubsystem extends SubsystemBase
{
    private Optional<EstimatedRobotPose> lastPose;
    private Field2d field;

    private PhotonPoseEstimator estimator;
    private PhotonCamera camera;

    public VisionSubsystem()
    {
        var robotToCamera = new Transform3d(0.5, 0.5, 0.5, new Rotation3d(0, 0, 0));

        try
        {
            var fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            camera = new PhotonCamera("USB_Camera");
            estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
        }
        catch (Exception exc)
        {
            System.out.println("Failed to initialize Vision!");
        }

        estimator.setReferencePose(new Pose2d(0, 0, new Rotation2d(0)));
        lastPose = estimator.update();
        field = new Field2d();
    }

    @Override
    public void periodic()
    {
        if (estimator != null)
        {
            if (lastPose.isPresent())
            {
                estimator.setReferencePose(lastPose.get().estimatedPose);
            }

            lastPose = estimator.update();
        }

        if (lastPose.isPresent())
        {
            var pose = lastPose.get().estimatedPose;
            var translation = pose.getTranslation();
            var rotation = pose.getRotation();

            SmartDashboard.putNumberArray("Robot translation", new double[] {translation.getX(), translation.getY(), translation.getZ()});

            SmartDashboard.putNumberArray("Robot rotation", new double[] {rotation.getX(), rotation.getY(), rotation.getZ()});

            field.setRobotPose(pose.toPose2d());
            SmartDashboard.putData("Simulated field", field);
        }
    }

    public PhotonTrackedTarget getTargets()
    {
        var result = camera.getLatestResult();
        if (result.hasTargets())
        {
            return result.getBestTarget();
        }

        return null;
    }

    public Command doAutoAlign()
    {
        return new AutoAlignCommand().andThen(new PrintCommand("AUTO-ALIGN: It works!"));
    }
}
