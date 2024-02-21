package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPositionPathPlanner extends Command {
    private Supplier<Pose2d> currentPose;
    private Rotation2d targetRotation;
    private Pose2d taregtPose;
    private SwerveSubsystem swerve;
    private List<Translation2d> bezierPoints;

    private PIDConstants translationConstants = new PIDConstants(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD);
    private PIDConstants rotationConstants = new PIDConstants(AutoConstants.kRotationP, AutoConstants.kRotationI, AutoConstants.kRotationD);

    public DriveToPositionPathPlanner(SwerveSubsystem drive, Supplier<Pose2d> currentPoseSupplier, Pose2d targetPosition) {
        addRequirements(drive);
        currentPose = currentPoseSupplier;
        taregtPose = targetPosition;
        bezierPoints = PathPlannerPath.bezierFromPoses(
            currentPoseSupplier.get(),
            targetPosition);
        targetRotation = targetPosition.getRotation();
        swerve = drive;

        System.out.println("Constructing Path");
    }

    @Override
    public void execute() {
        System.out.println("Moving to " + bezierPoints.get(bezierPoints.size()-1));
        new FollowPathHolonomic(
            new PathPlannerPath(bezierPoints,
             new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared, AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
             new GoalEndState(0, targetRotation)),
            () -> swerve.getPose(),
            swerve::getChassisSpeeds,
            swerve::driveRobotRelative,
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    translationConstants, // Translation PID constants
                    rotationConstants, // Rotation PID constants
                    AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                    Math.sqrt((SwerveConstants.wheelBase / 2)*(SwerveConstants.wheelBase / 2)*2), // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            swerve
        ).schedule();
    }

    @Override
    public boolean isFinished() {
        if ((currentPose.get().getTranslation().getDistance(taregtPose.getTranslation()) <= SwerveConstants.kAllowedDistanceToDestination)
            && (Math.abs(currentPose.get().getRotation().getRadians() - targetRotation.getRadians()) <= SwerveConstants.kAllowedRotationDifferenceToDestination)) {
                    System.out.println("Finishing movement");
                    return true; // Yes, I know I could just put the conditional in the return statement, but I need the print statement for testing
        }

        else if (swerve.getChassisSpeeds().equals(new ChassisSpeeds(0,0,0))) {
            System.out.println("Finishing movement");
            return true;
        }

        else {
            return false;
        }
    }

}
