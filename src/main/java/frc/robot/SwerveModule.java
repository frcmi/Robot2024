package frc.robot;

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
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    public Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private SwerveModuleState setState;

    public boolean isInverted;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    /* shuffleboard entries */
    private final UltraDoubleLog CANCoderPublisher;
    private final UltraDoubleLog anglePublisher;
    private final UltraDoubleLog velocityPublisher;

    private final SwerveModulePosition simulatedPosition = new SwerveModulePosition(0, new Rotation2d(0));


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

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    
        mDriveMotor.setInverted(isInverted);

        String modName;
        switch (moduleNumber) {
            case 0: {
                modName = "Front Left";
                break;
            }
            case 1: {
                modName = "Front Right";
                break;
            }
            case 2: {
                modName = "Back Left";
                break;
            }
            case 3: {
                modName = "Back Right";
                break;
            }
            default: {
                modName = "Mod " + moduleNumber;
                System.err.println("UNKNOWN SWERVE MODULES " + moduleNumber + ", module should be between 0 and 3");
            }
        }

        CANCoderPublisher = new UltraDoubleLog("Swerve/" + modName + "/CANCoder");
        anglePublisher = new UltraDoubleLog("Swerve/" + modName + "/Angle");
        velocityPublisher = new UltraDoubleLog("Swerve/" + modName + "/Velocity");
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
     * Sets the drive motor of the module to a speed, preferable to use {@link frc.robot.SwerveModule.setDesiredState}
     * @param desiredState
     * @param isOpenLoop
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
     * @return the reading of the CANcoder as a rotation2d
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
         CANCoderPublisher.update(getCANcoderReading().getDegrees());
         anglePublisher.update(getPosition().angle.getDegrees());
         velocityPublisher.update(getState().speedMetersPerSecond);
    }
}