package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.math.Conversions;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.lib.ultralogger.UltraTempLog;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    public Rotation2d angleOffset;

    public final TalonFX mAngleMotor;
    private final StatusSignal<Double> angleTemperatureSignal;
    public final TalonFX mDriveMotor;
    private final StatusSignal<Double> driveTemperatureSignal;
    private final CANcoder angleEncoder;
    private SwerveModuleState setState;

    public boolean isInverted;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    private final SwerveModulePosition simulatedPosition = new SwerveModulePosition(0, new Rotation2d(0));

    private final UltraTempLog driveTemperaturePublisher;
    private final UltraTempLog angleTemperaturePublisher;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, boolean isInverted){
        this.isInverted = isInverted;
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();
        mAngleMotor.setInverted(isInverted);
        angleTemperatureSignal = mAngleMotor.getDeviceTemp();

        angleTemperaturePublisher = new UltraTempLog("Swerve/Mod " + moduleNumber + "/Angle Motor Temperature", angleTemperatureSignal.asSupplier());

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
        mDriveMotor.setInverted(isInverted);
        
        driveTemperatureSignal = mDriveMotor.getDeviceTemp();

        driveTemperaturePublisher = new UltraTempLog("Swerve/Mod " + moduleNumber + "/Drive Motor Temperature", driveTemperatureSignal.asSupplier());
    }

    /**
     * Sets the desired state of the module
     * @param desiredState the state the module should be
     * @param isOpenLoop whether the module should use open or closed loop control
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
        setState = desiredState;
    }
    public SwerveModuleState getSetState() {
        return setState;
    }

    /**
     * Sets the drive motor of the module to a speed, preferable to use {@link #setDesiredState}
     * @param desiredState the state the module should be
     * @param isOpenLoop whether the module should use open or closed loop control
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    /**
     * @return the reading of the {@link CANcoder} as a {@link Rotation2d}
     */
    public Rotation2d getCANcoderReading(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    /**
     * Sets the module to face wheels forward
     */
    public void resetToAbsolute(){
        double absolutePosition = getCANcoderReading().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    /**
     * @return the current state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    /**
     * @return the position of the module based on measured values
     */
    public SwerveModulePosition getPosition() {
        if (RobotBase.isReal()) {
            return new SwerveModulePosition(
                    Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.SwerveConstants.wheelCircumference),
                    Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
            );
        } else {
            if (setState != null) {
                simulatedPosition.distanceMeters += setState.speedMetersPerSecond * 0.02;
                simulatedPosition.angle = setState.angle;
            }
            return simulatedPosition;
        }
    }

    /**
     * Logs all relevant values to shuffleboard. Should be called periodically.
     */
    public void logValues() {
        driveTemperaturePublisher.update();
        angleTemperaturePublisher.update();
    }
}