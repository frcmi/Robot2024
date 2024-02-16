package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    /* shuffleboard entries */
    private final ShuffleboardTab shuffleboardTab;
    private final GenericEntry CANCoderShuffleBoardItem;
    private final GenericEntry angleShuffleBoardItem;
    private final GenericEntry velocityShuffleBoardItem;


    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, boolean isInverted){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
        mDriveMotor.setInverted(isInverted);

        String modName;
        switch (moduleNumber) {
            case 0: {
                modName = "Front Left Swerve";
                break;
            }
            case 1: {
                modName = "Front Right Swerve";
                break;
            }
            case 2: {
                modName = "Back Left Swerve";
                break;
            }
            case 3: {
                modName = "Back Right Swerve";
                break;
            }
            default: {
                modName = "Unknown Swerve Mod " + moduleNumber;
                System.err.println("UNKNOWN SWERVE MODULES " + moduleNumber + ", module should be between 0 and 3");
            }
        }

        shuffleboardTab = Shuffleboard.getTab(modName);
        CANCoderShuffleBoardItem = shuffleboardTab.add("CANCoder", 0).withSize(2,2).withWidget(BuiltInWidgets.kGyro).withPosition(2,0).getEntry();
        angleShuffleBoardItem = shuffleboardTab.add("Angle", 0).withSize(2,2).withWidget(BuiltInWidgets.kGyro).withPosition(0,0).getEntry();
        velocityShuffleBoardItem = shuffleboardTab.add("Velocity", 0).withSize(4,1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", -SwerveConstants.maxSpeed, "Max", SwerveConstants.maxSpeed)).withPosition(0, 2).getEntry();
    }

    /**
     * Sets the desired state of the module
     * @param desiredState the state the module should be
     * @param isOpenLoop whether the module should use open or closed loop control
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the drive motor of the module to a speed, preferable to use {@link frc.robot.SwerveModule.SetDesiredState}
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
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    /**
     * @return the position of the module based on measured values
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    /**
     * Logs all relevant values to shuffleboard. Should be called periodically.
     */
    public void logValues() {
        CANCoderShuffleBoardItem.setDouble(getCANcoderReading().getDegrees());
        angleShuffleBoardItem.setDouble(getPosition().angle.getDegrees());
        velocityShuffleBoardItem.setDouble(getState().speedMetersPerSecond);
    }
}