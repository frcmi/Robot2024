package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private static final String cameraName = "USB_Camera";

    private Optional<EstimatedRobotPose> lastPose;
    private static Field2d field = new Field2d();

    private PhotonPoseEstimator estimator;
    private PhotonCamera camera;
    private final SwerveSubsystem swerve;

    /* shuffleboard entries */
    private static final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Vision");
    private static final GenericEntry xShuffleBoardItem = shuffleboardTab.add("X", 0).getEntry();
    private static final GenericEntry yShuffleBoardItem = shuffleboardTab.add("Y", 0).getEntry();
    private static final GenericEntry zShuffleBoardItem = shuffleboardTab.add("Z", 0).getEntry();
    private static final GenericEntry pitchShuffleBoardItem = shuffleboardTab.add("Pitch", 0).getEntry();
    private static final GenericEntry rollShuffleBoardItem = shuffleboardTab.add("Roll", 0).getEntry();
    private static final GenericEntry yawShuffleBoardItem = shuffleboardTab.add("Yaw", 0).getEntry();

    static {
        shuffleboardTab
                .add("Field", field)
                .withWidget(BuiltInWidgets.kField)
                .withSize(6,3);
    }

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

            xShuffleBoardItem.setDouble(translation.getX());
            yShuffleBoardItem.setDouble(translation.getY());
            zShuffleBoardItem.setDouble(translation.getZ());

            pitchShuffleBoardItem.setDouble(rotation.getX());
            rollShuffleBoardItem.setDouble(rotation.getY());
            yawShuffleBoardItem.setDouble(rotation.getZ());

            var pose2d = pose.toPose2d();
           // swerve.swerveDrivePoseEstimator.addVisionMeasurement(pose2d, MathSharedStore.getTimestamp());

            field.setRobotPose(pose2d);
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
