package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPositionPathPlanner {
    private Supplier<Pose2d> currentPose;
    private Rotation2d targetRotation;
    private Pose2d targetPose;
    private SwerveSubsystem swerve;
    private List<Translation2d> bezierPoints;

    private PIDConstants translationConstants = new PIDConstants(AutoConstants.kAccelerationP, AutoConstants.kAccelerationI, AutoConstants.kAccelerationD);
    private PIDConstants rotationConstants = new PIDConstants(AutoConstants.kRotationP, AutoConstants.kRotationI, AutoConstants.kRotationD);

    public DriveToPositionPathPlanner(SwerveSubsystem drive, Supplier<Pose2d> currentPoseSupplier, Pose2d targetPosition) {
        // addRequirements(drive);
        currentPose = currentPoseSupplier;
        this.targetPose = targetPosition;
        bezierPoints = PathPlannerPath.bezierFromPoses(
            currentPoseSupplier.get(),
            targetPosition);
        targetRotation = targetPosition.getRotation();
        swerve = drive;

        System.out.println("Constructing Path");
    }

    public Command gimmeCommand() { 
        System.out.println("Moving to " + bezierPoints.get(bezierPoints.size()-1));
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)
        );

// Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
    }

    // @Override
    // public boolean isFinished() {
    //     if ((currentPose.get().getTranslation().getDistance(taregtPose.getTranslation()) <= SwerveConstants.kAllowedDistanceToDestination)
    //         && (Math.abs(currentPose.get().getRotation().getRadians() - targetRotation.getRadians()) <= SwerveConstants.kAllowedRotationDifferenceToDestination)) {
    //                 System.out.println("Finishing movement");
    //                 return true; // Yes, I know I could just put the conditional in the return statement, but I need the print statement for testing
    //     }

    //     else if (swerve.getChassisSpeeds().equals(new ChassisSpeeds(0,0,0))) {
    //         System.out.println("Finishing movement");
    //         return true;
    //     }

    //     else {
    //         return false;
    //     }
    // }

}
