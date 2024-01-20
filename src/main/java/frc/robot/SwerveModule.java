package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.math.Conversions;
import frc.robot.Constants.SwerveConstants;

/**
 * Class for containing data perlative to a swerve module. Intended for one class per module
 */
public class SwerveModule {
    public final SwerveModuleConstants moduleConstants;
    public int moduleNumber;
    public Rotation2d angleOffset;
    protected Rotation2d previousAngle;

    protected TalonFX steerMotor;
    protected TalonFX driveMotor;
    protected CANcoder steerEncoder;

    protected SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(SwerveConstants.kDriveS, SwerveConstants.kDriveV, SwerveConstants.kDriveA);

    /**
     * @param moduleNumber the ID of this module
     * @param angleOffset the offset on the steer motor encoder
     * @param moduleConstants the constants of the module
     */
    public SwerveModule(int moduleNumber, Rotation2d angleOffset, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        this.moduleConstants = moduleConstants;

        steerMotor = new TalonFX(moduleConstants.SteerMotorId);
        driveMotor = new TalonFX(moduleConstants.DriveMotorId);
        steerEncoder = new CANcoder(moduleConstants.CANcoderId);
    }

    public void setSteerMotor(double speed) {
        steerMotor.set(speed);
    }

    /**
     * @return gets the angle the steer motor is pointed at
     */
    public Rotation2d getSteerMotorAngle() {
        return Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * stops the drive motor from moving forward
     */
    public void stopDriveMotor() {
        driveMotor.stopMotor();
    }

    public void setCoastMode() {
        steerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * @return the position of the module relative to the center
     */
    public Translation2d getPosition() {
        return new Translation2d(moduleConstants.LocationX, moduleConstants.LocationY);
    }

    /**
     * @return the position of the module relative to the center
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getPosition().getNorm(), getPosition().getAngle());
    }
}
