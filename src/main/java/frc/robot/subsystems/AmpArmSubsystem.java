package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ultralogger.UltraBooleanLog;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.lib.ultralogger.UltraTempLog;
import frc.robot.Constants.AmpArmConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class AmpArmSubsystem extends SubsystemBase{
    public final CANSparkMax armMotor = new CANSparkMax(AmpArmConstants.kAmpArmMotorId, MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(AmpArmConstants.kArmEncoderId);

    // PID Controller for arm movement
    private final ProfiledPIDController pidController 
        = new ProfiledPIDController(AmpArmConstants.kP, AmpArmConstants.kI, AmpArmConstants.kD, 
            new TrapezoidProfile.Constraints(AmpArmConstants.kMaxArmVel, AmpArmConstants.kMaxArmAccel));

    private final ArmFeedforward feedforward = 
        new ArmFeedforward(AmpArmConstants.kS, AmpArmConstants.kG, AmpArmConstants.kV, AmpArmConstants.kA);

    private final UltraDoubleLog radianPublisher = new UltraDoubleLog("Amp Arm/Radians");
    private final UltraDoubleLog currentPublisher = new UltraDoubleLog("Amp Arm/Motor Current");
    private final UltraDoubleLog goalAnglePublisher = new UltraDoubleLog("Amp Arm/Goal Angle");
    private final UltraBooleanLog boundsPublisher = new UltraBooleanLog("Amp Arm/Bounds");
    private final UltraTempLog temperaturePublisher = new UltraTempLog("Amp Arm/Motor Temperature", armMotor::getMotorTemperature);

    public AmpArmSubsystem() {
        armEncoder.setDistancePerRotation(1);
        armEncoder.setPositionOffset(AmpArmConstants.kAmpEncoderOffset / 360);
        armMotor.setIdleMode(IdleMode.kBrake);

        armMotor.setInverted(false);

        setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        radianPublisher.update(getAngle());
        currentPublisher.update(armMotor.getOutputCurrent());
        temperaturePublisher.update();
    }

    /**
     * Return the current angle in RADIANS
     */
    public double getAngle() {
        return -((armEncoder.getAbsolutePosition()) * 2 * Math.PI) + Math.toRadians(AmpArmConstants.kAmpEncoderOffset); // Angle is in radians
    }     

    public void setGoal(double goalAngle) {
        double angle = getAngle();
        double kg = feedforward.calculate(angle, 0);

        goalAngle = Math.toRadians(goalAngle);
        
         goalAnglePublisher.update(goalAngle);
         boundsPublisher.update(!(angle > AmpArmConstants.kMaxAngle || angle < AmpArmConstants.kMinAngle));

        double pidOutput = pidController.calculate(angle, goalAngle);
       // double ffOutput = feedforward.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity);

        double outputVolts = pidOutput + /*ffOutput +*/ Math.cos(angle) * (goalAngle < AmpArmConstants.kGravityLimit ? 0 : AmpArmConstants.kTorqueArmConstant);

        // Stop movement if outside bounds
        if (angle < AmpArmConstants.kMinAngle) 
            outputVolts = Math.max(kg, Math.min(2, outputVolts));
        if (angle > AmpArmConstants.kMaxAngle)
            outputVolts = Math.max(-2, Math.min(kg, outputVolts));

        outputVolts = Math.max(-AmpArmConstants.kMaxArmVolts, Math.min(AmpArmConstants.kMaxArmVolts, outputVolts));

        armMotor.setVoltage(outputVolts);
    }

    
    public Command moveTo(double goalAngle) {
        return run(() -> setGoal(goalAngle));
    }

    public Command raiseToAmp() {
        double raiseVolts = AmpArmConstants.kRaiseArmVolts + Math.cos(getAngle()) * AmpArmConstants.kTorqueArmConstant;
        raiseVolts = Math.max(-AmpArmConstants.kMaxArmVolts, Math.min(AmpArmConstants.kMaxArmVolts, raiseVolts));
        final double outputVolts = raiseVolts;

        return new PrintCommand("Raising Arm")
            .andThen(run(() -> armMotor.setVoltage(AmpArmConstants.kRaiseArmVolts + Math.cos(getAngle()) * AmpArmConstants.kTorqueArmConstant)))
            .until(() -> (Math.toDegrees(getAngle()) > 92))
            .andThen(runOnce(() -> System.out.println("At " + Math.toDegrees(getAngle()) +", stopping now")))
            .andThen(stop());
    }

    public Command lowerArm() {
        return run(
                () -> armMotor.setVoltage(AmpArmConstants.kLowerArmVolts)
        ).until(
                () -> armMotor.getOutputCurrent() > AmpArmConstants.kAmpCurrentLimit
        );
    }
      
    public Command stop() { //TODO: can change
        return run (
            () -> { 
                armMotor.set(0);
            }
        ).withName("stop");
    }
}
    
