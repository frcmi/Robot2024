package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrive swerveDrivetrain;
    
    public Pigeon2 gyro = new Pigeon2(SwerveConstants.kPigeonId);

    public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d[] {
            SwerveConstants.Mod0.getPosition(),
            SwerveConstants.Mod1.getPosition(),
            SwerveConstants.Mod2.getPosition(),
            SwerveConstants.Mod3.getPosition()
        });

    public SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        kinematics, 
        gyro.getRotation2d(), 
        new SwerveModulePosition[]{
            SwerveConstants.Mod0.getSwerveModulePosition(),
            SwerveConstants.Mod1.getSwerveModulePosition(),
            SwerveConstants.Mod2.getSwerveModulePosition(),
            SwerveConstants.Mod3.getSwerveModulePosition()
        },
        new Pose2d(
            0,
            0,
            new Rotation2d(0))
        );

    public SwerveModule[] swerveModules = {
        SwerveConstants.Mod1,
        SwerveConstants.Mod0,
        SwerveConstants.Mod2,
        SwerveConstants.Mod3
    };

    public SwerveSubsystem() {
        resetGyro();
        try {
            swerveDrivetrain = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(SwerveConstants.kMaxSpeed);
        } catch (Exception e) {
            System.err.println("YAGSL swerve configuration file not found! This is a very big problem and will make swerve not work! Expect a null reference after you see this!");
        }
    }

    @Override
    public void periodic() {
        odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                swerveModules[0].getSwerveModulePosition(),
                swerveModules[1].getSwerveModulePosition(),
                swerveModules[2].getSwerveModulePosition(),
                swerveModules[3].getSwerveModulePosition(),
            });
    }

    public Command driveFieldCentric(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier spinVelocity) {
        return runOnce(() -> {
            swerveDrivetrain.drive(
                new Translation2d(
                    xVelocity.getAsDouble(),
                    yVelocity.getAsDouble()),
                spinVelocity.getAsDouble(),
                true,
                false);
            SmartDashboard.putNumber("Distance", swerveDrivetrain.getModulePositions()[0].distanceMeters);
        });
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
