package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPosition extends Command {
    SwerveSubsystem m_swerveSubsystem;
    Supplier<Pose2d> currentPosition;
    Pose2d destination;
    PIDController xPID = new PIDController(
        SwerveConstants.AutoConstants.kAutoXP,
        SwerveConstants.AutoConstants.kAutoXI,
        SwerveConstants.AutoConstants.kAutoXD);
    PIDController yPID = new PIDController(
        SwerveConstants.AutoConstants.kAutoYP,
        SwerveConstants.AutoConstants.kAutoYI,
        SwerveConstants.AutoConstants.kAutoYD);
    PIDController turnPID = new PIDController(
        SwerveConstants.AutoConstants.kRotationP,
        SwerveConstants.AutoConstants.kRotationI,
        SwerveConstants.AutoConstants.kRotationD);

    public DriveToPosition(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> currentPosition, Pose2d destination) {
        m_swerveSubsystem = swerveSubsystem;
        this.currentPosition = currentPosition;
        this.destination = destination;
        xPID.setSetpoint(destination.getX());
        yPID.setSetpoint(destination.getY());
        turnPID.setSetpoint(destination.getRotation().getRadians());
    }

    @Override
    public void execute() {
        m_swerveSubsystem.driveFieldCentric(
            xPID.calculate(currentPosition.get().getX()), 
            yPID.calculate(currentPosition.get().getY()), 
            turnPID.calculate(currentPosition.get().getRotation().getRadians()));
    }

    @Override
    public boolean isFinished() {
        Translation2d t = currentPosition.get().minus(destination).getTranslation();
        return Math.sqrt(t.getX()*t.getX() + t.getY()*t.getY()) <= SwerveConstants.kAllowedDistanceToDestination;
    }
    
}
