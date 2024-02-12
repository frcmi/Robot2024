package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Transformations;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
/**
 * Command that drives the robot to a field position
 */
public class DriveToPosition extends Command {
    SwerveSubsystem swerve;
    Supplier<Pose2d> positionSupplier;
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

    StructPublisher<Pose2d> positionPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("DriveToPosition", Pose2d.struct).publish();

    /**
     * Drive robot to a given position using PID
     * @param swerveSubsystem the swerve subsystem to spin motors on
     * @param currentPosition a supplier that contains the consitently updated position of the bot
     * @param destination the position that the robot should drive to, relative to the field
     */
    public DriveToPosition(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> currentPosition, Pose2d dst) {
        addRequirements(swerveSubsystem);
        // SmartDashboard.setDefaultNumber("move pid P", 0.2);
        // SmartDashboard.setDefaultNumber("move pid I", 0);
        // SmartDashboard.setDefaultNumber("move pid D", 0);
        swerve = swerveSubsystem;
        
        positionSupplier = currentPosition;
        destination = dst;

        xPID.setSetpoint(destination.getX());
        yPID.setSetpoint(destination.getY());
        turnPID.setSetpoint(destination.getRotation().getRadians());
        positionPublisher.set(destination);
        
        SmartDashboard.putData("Move PID", xPID);
        SmartDashboard.putData("Rotation PID", turnPID);

        addRequirements(swerveSubsystem);
    }

    StructPublisher<Pose2d> setPositionPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("set position", Pose2d.struct).publish();

    @Override
    public void execute() {
        setPositionPublisher.set(destination);
        xPID = (PIDController) SmartDashboard.getData("Move PID");
        turnPID = (PIDController) SmartDashboard.getData("Rotation PID");
        yPID.setPID(xPID.getP(), xPID.getI(), xPID.getD());
        
        var pose = positionSupplier.get();
        double x = xPID.calculate(pose.getX());
        double y = yPID.calculate(pose.getY());
        double rotation = turnPID.calculate(pose.getRotation().getRadians());

        var translation = new Translation2d(x, y);
        swerve.drive(translation, rotation, true, false);
    }

    @Override
    public boolean isFinished() {
        var toDestination = positionSupplier.get().minus(destination);
        return Transformations.length(toDestination.getTranslation()) <= SwerveConstants.kAllowedDistanceToDestination &&
               Math.abs(toDestination.getRotation().getRadians()) <= SwerveConstants.kAllowedRotationDifferenceToDestination;
    }
    
}
