package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.ultralogger.UltraStructLog;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
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

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
        //21.85 up

        try
        {
            var fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            camera = new PhotonCamera(cameraName);
            estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.robotToCamera);

            if (Robot.isSimulation()) {
                // Create the vision system simulation which handles cameras and targets on the field.
                visionSim = new VisionSystemSim("Arducam_OV9281_USB_Camera");
                // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
                visionSim.addAprilTags(fieldLayout);
                // Create simulated camera properties. These can be set to mimic your actual camera.
                // TODO: Make sure these are the actual values
                var cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
                cameraProp.setCalibError(0.44, 0.05);
                cameraProp.setFPS(30);
                cameraProp.setAvgLatencyMs(30);
                cameraProp.setLatencyStdDevMs(10);
                // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
                // targets.
                cameraSim = new PhotonCameraSim(camera, cameraProp);
                // Add the simulated camera to view the targets on this simulated field.
                visionSim.addCamera(cameraSim, VisionConstants.robotToCamera);

                cameraSim.enableDrawWireframe(true);
            }
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

    @Override
    public void simulationPeriodic() {
        visionSim.update(swerve.swerveDriveOdometrySim.getPoseMeters());
        SmartDashboard.putData("Vision Sim", visionSim.getDebugField());
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }
}
