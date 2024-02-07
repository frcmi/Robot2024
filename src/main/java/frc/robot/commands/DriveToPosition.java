package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Transformations;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
/**
 * Command that drives the robot to a field position
 */
public class DriveToPosition extends Command {
    SwerveSubsystem m_swerveSubsystem;
    Supplier<Pose2d> currentPosition;
    Pose2d destination;
    PIDController xPID = new PIDController(
        AutoConstants.kAutoXP,
        AutoConstants.kAutoXI,
        AutoConstants.kAutoXD);
    PIDController yPID = new PIDController(
        AutoConstants.kAutoYP,
        AutoConstants.kAutoYI,
        AutoConstants.kAutoYD);
    PIDController turnPID = new PIDController(
        AutoConstants.kRotationP,
        AutoConstants.kRotationI,
        AutoConstants.kRotationD);

    StructPublisher<Pose2d> setPosPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("DriveToPosition", Pose2d.struct).publish();

    /**
     * Drive robot to a given position using PID
     * @param swerveSubsystem the swerve subsystem to spin motors on
     * @param currentPosition a supplier that contains the consitently updated position of the bot
     * @param destination the position that the robot should drive to, relative to the field
     */
    public DriveToPosition(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> currentPosition, Pose2d destination) {
        m_swerveSubsystem = swerveSubsystem;
        
        this.currentPosition = currentPosition;
        this.destination = destination;

        xPID.setSetpoint(destination.getX());
        yPID.setSetpoint(destination.getY());
        turnPID.setSetpoint(destination.getRotation().getRadians());

        setPosPublisher.set(destination);
    }

    @Override
    public void execute() {
        double x = xPID.calculate(currentPosition.get().getX());
        double y = yPID.calculate(currentPosition.get().getY());
        double rotation = turnPID.calculate(currentPosition.get().getRotation().getRadians());

        var translation = new Translation2d(x, y);
        m_swerveSubsystem.drive(translation, rotation, true, false);
    }

    @Override
    public boolean isFinished() {
        var toDestination = currentPosition.get().minus(destination);

        return Transformations.length(toDestination.getTranslation()) <= SwerveConstants.kAllowedDistanceToDestination &&
               Math.abs(toDestination.getRotation().getRadians()) <= SwerveConstants.kAllowedRotationDifferenceToDestination;
    }
    
}
