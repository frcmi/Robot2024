package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
    //TODO: delete this once vision is finished
    public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d[] {
        SwerveConstants.Mod0.getPosition(),
        SwerveConstants.Mod1.getPosition(),
        SwerveConstants.Mod2.getPosition(),
        SwerveConstants.Mod3.getPosition()
    });
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
        SwerveConstants.Mod3.moduleConstants
    );
    public SwerveModule[] swerveModules = {
        SwerveConstants.Mod0,
        SwerveConstants.Mod1,
        SwerveConstants.Mod2,
        SwerveConstants.Mod3
    };
    //TODO: can be removed once vision is finished
    public SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        kinematics, 
        gyro.getRotation2d(), new SwerveModulePosition[]{
            SwerveConstants.Mod0.getSwerveModulePosition(),
            SwerveConstants.Mod1.getSwerveModulePosition(),
            SwerveConstants.Mod2.getSwerveModulePosition(),
            SwerveConstants.Mod3.getSwerveModulePosition()
        });

    public SwerveSubsystem() {
        resetGyro();
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setCoastMode();
        }
    }

    @Override
    public void periodic() {
        updateChassisSpeeds();
        for (int i = 0; i < swerveModules.length; i++) {
            SmartDashboard.putNumber("Module " + swerveModules[i].moduleNumber + " Encoder Rotation", swerveModules[i].getSteerMotorAngle().getRadians());
        }
    }
    public Command test() {
        return runOnce(() -> {
            swerveModules[0].setSteerMotor(0.1);
        });
    }

    /**
     * Drive relative to the field.
     * @param forwardVelocity the velocity the robot should go towards the end of the field
     * @param sidewaysVelocity the velocity the robot should go towards the left side of the field
     * @param rotationAboutCenter the velocity of the robot around it's center
     */
    public Command driveFieldCentric(DoubleSupplier forwardVelocity, DoubleSupplier sidewaysVelocity, DoubleSupplier rotationAboutCenter) {
        return runOnce(() -> {
            swerveDrivetrain.setControl(getRequestFieldCentric(forwardVelocity.getAsDouble(), sidewaysVelocity.getAsDouble(), rotationAboutCenter.getAsDouble()));
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
    /**
     * @param chassisSpeeds the speeds at which to run the bot
     */
    // public void driveChassisSpeedsFieldCentric(ChassisSpeeds chassisSpeeds) {
    //     driveFieldCentric(
    //         chassisSpeeds.vxMetersPerSecond, 
    //         chassisSpeeds.vyMetersPerSecond, 
    //         chassisSpeeds.omegaRadiansPerSecond);
    // }
    /**
     * To be ran once per gyro update.
     */
    private void updateChassisSpeeds() {
        chassisSpeeds.omegaRadiansPerSecond = gyro.getRate();
        chassisSpeeds = chassisSpeeds.plus(
            new ChassisSpeeds(
                gyro.getAccelerationX().getValueAsDouble(), 
                gyro.getAccelerationY().getValueAsDouble(), 
                0d));
    }
    /**
     * @return the speeds at which the chassises of the robot are going
     */
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
