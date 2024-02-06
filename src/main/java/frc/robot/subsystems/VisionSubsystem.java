package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private static final String cameraName = "USB_Camera";

    private Optional<EstimatedRobotPose> lastPose;
    private Field2d field;

    private PhotonPoseEstimator estimator;
    private PhotonCamera camera;
    private final SwerveSubsystem swerve;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
        var robotToCamera = new Transform3d(0.3048, 0.0254, 0.5588, new Rotation3d(0, 0, Math.PI));

        try
        {
            var fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            camera = new PhotonCamera(cameraName);
            estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
        }
        catch (Exception exc)
        {
            System.out.println("Failed to initialize Vision!");
        }

        lastPose = estimator.update();
        field = new Field2d();
        swerve = swerveSubsystem;
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

            SmartDashboard.putNumber("Vision X", translation.getX());
            SmartDashboard.putNumber("Vision Y", translation.getY());
            SmartDashboard.putNumber("Vision Z", translation.getZ());

            SmartDashboard.putNumber("Vision pitch", rotation.getX());
            SmartDashboard.putNumber("Vision roll", rotation.getY());
            SmartDashboard.putNumber("Vision yaw", rotation.getZ());

            var pose2d = pose.toPose2d();
            swerve.swerveDrivePoseEstimator.addVisionMeasurement(pose2d, MathSharedStore.getTimestamp());

            field.setRobotPose(pose2d);
            SmartDashboard.putData("Vision field", field);
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
