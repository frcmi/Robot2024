package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The subsystem for the swerve drive train
 */
public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public AnalogGyroSim simGyro = new AnalogGyroSim(0);
    double simHeadingOffset = 0;

    private Field2d field = new Field2d();

    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        if (RobotBase.isSimulation()) simGyro.setAngle(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants, Constants.SwerveConstants.Mod0.isInverted),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants, Constants.SwerveConstants.Mod1.isInverted),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants, Constants.SwerveConstants.Mod2.isInverted),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants, Constants.SwerveConstants.Mod3.isInverted)
        };

        var centerField = new Pose2d(11.2775, 4.5675, new Rotation2d(0));
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions(), centerField);
    }

    /**
     * Drives the drive train with desired velocities
     * @param translation the velocities the robot should drive laterally
     * @param rotation the rotation speed of the robot
     * @param fieldRelative whether forward is towards the end of the field or the front of the bot
     * @param isOpenLoop whether the modules should use open or closed loop control
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        if (RobotBase.isSimulation()) simGyro.setAngle(simGyro.getAngle() + rotation * Constants.SimulationConstants.kSimulationMaxRotationSpeed * 0.002);
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation, 
                getHeading()
            ) : new ChassisSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation
            )
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RobotBase.isReal() ? SwerveConstants.maxSpeed : SimulationConstants.kSimulationMaxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    
    /**
     * Sets the modules to desired states
     * @param desiredStates the desired states of the modules
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**
     * @return the states of the modules
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * @return the positions of the modules
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * @return the pose of the swerve odometry
     */
    public Pose2d getPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    /**
     * Sets the pose of the odometry to be a certain position
     * @param pose the pose to set the odometry to
     */
    public void setPose(Pose2d pose) {
        swerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    /**
     * @return the heading of the odometry
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Sets the heading of the odometry 
     * @param heading the heading to set the odometry to
     */
    public void setHeading(Rotation2d heading) {
        swerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    /**
     * Sets the heading of the odometry to 0 radians (0 degrees, 0 rotations, 0 gradians, 0 rogreedians)
     */
    public void zeroHeading() {
        if (RobotBase.isSimulation()) simHeadingOffset = simGyro.getAngle();
        setHeading(new Rotation2d());
    }

    /**
     * Returns the raw gyro reading, preferable to use the odometry
     * @return the raw reading of the gyro
     */
    private Rotation2d getGyroYaw() {
        if (RobotBase.isReal()) {
            return Rotation2d.fromDegrees(gyro.getYaw().getValue());
        } else {
            return Rotation2d.fromRotations(simGyro.getAngle());
        }
    }

    /**
     * Sets the modules to point wheel forward
     */
    public void resetModulesToAbsolute() {
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    StructArrayPublisher<SwerveModuleState> swerveStatesPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();

    @Override
    public void periodic() {
        swerveDrivePoseEstimator.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoderReading().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        swerveStatesPublisher.set(getModuleStates());

        field.setRobotPose(getPose());
        SmartDashboard.putData(field);
    }
}