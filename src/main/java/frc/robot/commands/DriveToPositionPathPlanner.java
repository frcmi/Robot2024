package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPositionPathPlanner {
    private Pose2d targetPose;
    private List<Translation2d> bezierPoints;

    public DriveToPositionPathPlanner(Pose2d currentPose, Pose2d targetPosition) {
        // addRequirements(drive);
        this.targetPose = targetPosition;
        bezierPoints = PathPlannerPath.bezierFromPoses(
            currentPose,
            targetPosition
        );

        System.out.println("Constructing Path");
    }

    public Command getCommand() { 
        System.out.println("Moving to " + bezierPoints.get(bezierPoints.size()-1));
        PathConstraints constraints = new PathConstraints(
            1.5, 2.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)
        );

        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
    }
}
