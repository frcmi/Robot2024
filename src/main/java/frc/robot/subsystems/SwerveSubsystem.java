package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.lib.ultralogger.UltraStructArrayLog;
import frc.lib.ultralogger.UltraStructLog;
import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The subsystem for the swerve drive train
 */
public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public boolean sensitivitySwitch = false;
    public double translationSensitivity = 1;
    public double rotationSensitivity = 1;

    private PIDConstants translationConstants = new PIDConstants(AutoConstants.kAccelerationP,
            AutoConstants.kAccelerationI, AutoConstants.kAccelerationD);
    private PIDConstants rotationConstants = new PIDConstants(AutoConstants.kRotationP, AutoConstants.kRotationI,
            AutoConstants.kRotationD);

    public AnalogGyroSim simGyro = new AnalogGyroSim(0);
    double simHeadingOffset = 0;


    public SwerveSubsystem() {
        if (RobotBase.isSimulation()) simGyro.setAngle(0);
        gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.SwerveConstants.Mod0.constants,
                        Constants.SwerveConstants.Mod0.isInverted),
                new SwerveModule(1, Constants.SwerveConstants.Mod1.constants,
                        Constants.SwerveConstants.Mod1.isInverted),
                new SwerveModule(2, Constants.SwerveConstants.Mod2.constants,
                        Constants.SwerveConstants.Mod2.isInverted),
                new SwerveModule(3, Constants.SwerveConstants.Mod3.constants, Constants.SwerveConstants.Mod3.isInverted)
        };

        Pose2d centerField = new Pose2d(11.2775, 4.5675, new Rotation2d(0));
        Pose2d speakerStart = new Pose2d(15.27, 5.55, new Rotation2d(Math.toRadians(-180)));
        Pose2d station1 = new Pose2d(16.27, 7.07, new Rotation2d(Math.toRadians(180)));
        Pose2d blueStation = new Pose2d(0.53, 7.11, new Rotation2d(0));
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics,
                getGyroYaw(), getModulePositions(), blueStation);

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
                        // likely live in your Constants class
                        translationConstants, // Translation PID constants
                        rotationConstants, // Rotation PID constants
                        AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                        Math.sqrt((Units.inchesToMeters(SwerveConstants.wheelBase) /
                                2) * (Units.inchesToMeters(SwerveConstants.wheelBase) / 2) * 2), // Drive base
                        // radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Drives the drive train with desired velocities
     * 
     * @param translation   the velocities the robot should drive laterally
     * @param rotation      the rotation speed of the robot
     * @param fieldRelative whether forward is towards the end of the field or the
     *                      front of the bot
     * @param isOpenLoop    whether the modules should use open or closed loop
     *                      control
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        if (RobotBase.isSimulation()) simGyro.setAngle(simGyro.getAngle() + rotation * Constants.SimulationConstants.kSimulationMaxRotationSpeed * 0.002);
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RobotBase.isReal() ? Constants.SwerveConstants.maxSpeed : Constants.SimulationConstants.kSimulationMaxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    /**
     * Sets the modules to desired states
     * 
     * @param desiredStates the desired states of the modules
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }

    /**
     * @return the states of the modules
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModuleState[] getModuleSetpoints() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getSetState();
        }
        return states;
    }

    /**
     * @return the positions of the modules
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
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
     * 
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
     * 
     * @param heading the heading to set the odometry to
     */
    public void setHeading(Rotation2d heading) {
        swerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    /**
     * Sets the heading of the odometry to 0 radians (0 degrees, 0 rotations, 0
     * gradians, 0 rogreedians)
     */
    public void zeroHeading() {
        if (RobotBase.isSimulation()) simHeadingOffset = simGyro.getAngle();
        setHeading(new Rotation2d());
    }

    /**
     * Returns the raw gyro reading, preferable to use the odometry
     * 
     * @return the raw reading of the gyro
     */
    private Rotation2d getGyroYaw() {
        if (RobotBase.isReal()) {
            return Rotation2d.fromDegrees(gyro.getYaw().getValue());
        } else {
            return Rotation2d.fromRotations(simGyro.getAngle() + simHeadingOffset);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Sets the modules to point wheel forward
     */
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public Command stop() {
        return Commands.run(() -> driveRobotRelative(new ChassisSpeeds(0, 0, 0)), this);
    }

    /**
     * Toggles the sensitivity switch
     */
    public void switchSensitivity() {
        sensitivitySwitch = !sensitivitySwitch;
        translationSensitivity = sensitivitySwitch ? SwerveConstants.translationSensitivity : 1;
        rotationSensitivity = sensitivitySwitch ? SwerveConstants.rotationSensitivity : 1;
    }

    // UltraStructArrayLog<SwerveModuleState> swerveStatePublisher = new UltraStructArrayLog<>("Swerve/Current States", SwerveModuleState.struct);
    // UltraStructArrayLog<SwerveModuleState> swerveSetpointPublisher = new UltraStructArrayLog<>("Swerve/Set States", SwerveModuleState.struct);
    // UltraStructLog<Pose2d> posePublisher = new UltraStructLog<>("Swerve/Pose Estimator", Pose2d.struct);
    // UltraDoubleLog angularVelocityPublisher = new UltraDoubleLog("Swerve/Angular Velocity");

    @Override
    public void periodic() {
        swerveDrivePoseEstimator.update(getGyroYaw(), getModulePositions());
        // posePublisher.update(getPose());
        // swerveStatePublisher.update(getModuleStates());
        // swerveSetpointPublisher.update(getModuleSetpoints());
        // angularVelocityPublisher.update(gyro.getAngularVelocityZWorld().getValueAsDouble());
    }
}