package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.*;
import frc.lib.ultralogger.UltraStructLog;
import frc.robot.Constants.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private static final String cameraName = "Arducam_OV9281_USB_Camera";

    private Optional<EstimatedRobotPose> lastPose;

    private PhotonPoseEstimator estimator;
    private PhotonCamera camera;
    private final SwerveSubsystem swerve;

    private final UltraStructLog<Pose3d> posePublisher = new UltraStructLog<>("Vision/pose3d", Pose3d.struct);
    // Needed until https://github.com/Mechanical-Advantage/AdvantageScope/issues/149 is closed.
    private final UltraStructLog<Pose2d> pose2dPublisher = new UltraStructLog<>("Vision/pose2d", Pose2d.struct);

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
        //21.85 up

        try
        {
            var fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            camera = new PhotonCamera(cameraName);
            estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.robotToCamera);
        }
        catch (Exception exc)
        {
            System.out.println("Failed to initialize Vision!");
        }

        lastPose = estimator.update();
        swerve = swerveSubsystem;
    }

    @Override
    public void periodic()
    {
        // estimator.setLastPose(swerve.getPose());
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
            EstimatedRobotPose estimatedPoseObject = lastPose.get();
            Pose3d pose = estimatedPoseObject.estimatedPose;

            Pose2d pose2d = pose.toPose2d();
            posePublisher.update(pose);
            pose2dPublisher.update(pose2d);

            swerve.swerveDrivePoseEstimator.addVisionMeasurement(pose2d, estimatedPoseObject.timestampSeconds);
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
}
