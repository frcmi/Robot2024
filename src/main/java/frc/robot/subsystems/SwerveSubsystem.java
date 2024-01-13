package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrivetrainConstants swerveDrivetrainConstants = new SwerveDrivetrainConstants()
        .withPigeon2Id(SwerveConstants.kPigeonId)
        .withCANbusName(SwerveConstants.kCanbusName);
    public Pigeon2 gyro = new Pigeon2(SwerveConstants.kPigeonId); 
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    public SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(
        swerveDrivetrainConstants, 
        SwerveConstants.Mod0.moduleConstants,
        SwerveConstants.Mod1.moduleConstants,
        SwerveConstants.Mod2.moduleConstants,
        SwerveConstants.Mod3.moduleConstants);
    public SwerveModule[] swerveModules = {
        SwerveConstants.Mod0,
        SwerveConstants.Mod1,
        SwerveConstants.Mod2,
        SwerveConstants.Mod3
    };

    public SwerveSubsystem() {
        resetGyro();
    }

    @Override
    public void periodic() {
        updateChassisSpeeds();
    }

    /**
     * Drive relative to the field.
     * @param forwardVelocity the velocity the robot should go towards the end of the field
     * @param sidewaysVelocity the velocity the robot should go towards the left side of the field
     * @param rotationAboutCenter the velocity of the robot around it's center
     */
    public Command driveFieldCentric(double forwardVelocity, double sidewaysVelocity, double rotationAboutCenter) {
        return runOnce(() -> {
            swerveDrivetrain.setControl(getRequestFieldCentric(forwardVelocity, sidewaysVelocity, rotationAboutCenter));
        });
    }
    /**
     * Drive relative to the robot.
     * @param forwardVelocity the velocity the robot should go forward
     * @param sidewaysVelocity the velocity the robot should go towards the left
     * @param rotationAboutCenter the velocity of the robot around it's center
     */
    public Command driveRobotCentric(double forwardVelocity, double sidewaysVelocity, double rotationAboutCenter) {
        return runOnce(() -> {
            swerveDrivetrain.setControl(getRequestRobotCentric(forwardVelocity, sidewaysVelocity, rotationAboutCenter));
        });
    }
    /**
     * Drive relative to the field.
     * @param forwardVelocity the velocity the robot should go towards the end of the field
     * @param sidewaysVelocity the velocity the robot should go towards the left side of the field
     * @param rotationAboutCenter the velocity of the robot around it's center
     * @return a swerve request that can be modified before sending it to the swerve subsystem
     */
    public SwerveRequest getRequestFieldCentric(double forwardVelocity, double sidewaysVelocity, double rotationAboutCenter) {
        return new SwerveRequest.FieldCentric()
            .withVelocityX(forwardVelocity)
            .withVelocityY(sidewaysVelocity)
            .withRotationalRate(rotationAboutCenter);
    }
    /**
     * Drive relative to the robot.
     * @param forwardVelocity the velocity the robot should go forward
     * @param sidewaysVelocity the velocity the robot should go towards the left
     * @param rotationAboutCenterthe velocity of the robot around it's center
     * @return a swerve request that can be modified before sending it to the swerve subsystem
     */
    public SwerveRequest getRequestRobotCentric(double forwardVelocity, double sidewaysVelocity, double rotationAboutCenter) {
        return new SwerveRequest.RobotCentric()
            .withVelocityX(forwardVelocity)
            .withVelocityY(sidewaysVelocity)
            .withRotationalRate(rotationAboutCenter);
    }
    public void driveChassisSpeedsFieldCentric(ChassisSpeeds chassisSpeeds) {
        driveFieldCentric(
            chassisSpeeds.vxMetersPerSecond, 
            chassisSpeeds.vyMetersPerSecond, 
            chassisSpeeds.omegaRadiansPerSecond);
    }
    private void updateChassisSpeeds() {
        chassisSpeeds.omegaRadiansPerSecond = gyro.getRate();
        chassisSpeeds = chassisSpeeds.plus(
            new ChassisSpeeds(
                gyro.getAccelerationX().getValueAsDouble(), 
                gyro.getAccelerationY().getValueAsDouble(), 
                0d));
    }
    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }
    /**
     * Drive using a Phoenix6 SwerveRequest
     * @param driveRequest the SwerveRequest to give to the drive train
     */
    public void driveSwerveRequest(SwerveRequest driveRequest) {
        swerveDrivetrain.setControl(driveRequest);
    }

    /**
     * Set the yaw value of the gyro to 0
     */
    public void resetGyro() {
        gyro.setYaw(0);
    }

    /**
     * Stop all the drive motors.
     */
    public void stopMotors() {
        for (SwerveModule module : swerveModules) {
            module.stopDriveMotor();
        }
    }
}
