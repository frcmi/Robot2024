package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    public AmpArmSubsystem() {
        armEncoder.setDistancePerRotation(1);
        armEncoder.setPositionOffset(AmpArmConstants.kAmpEncoderOffset / 360);
        armMotor.setIdleMode(IdleMode.kBrake);

        armMotor.setInverted(false);

        setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Degrees", Math.toDegrees(getAngle()));
        // SmartDashboard.putNumber("Arm Radians", getAngle());
    }

    public double getAngle() {
        return -((armEncoder.getAbsolutePosition()) * 2 * Math.PI) + Math.toRadians(AmpArmConstants.kAmpEncoderOffset); // Angle is in radians
    }     

    public void setGoal(double goalAngle) {
        double angle = getAngle();
        double kg = feedforward.calculate(angle, 0);
        
        // SmartDashboard.putNumber("Goal Angle", goalAngle);
        // SmartDashboard.putBoolean("Arm Bounds", !(angle > AmpArmConstants.kMaxAngle || angle < AmpArmConstants.kMinAngle));

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
      
    public Command stop() { //TODO: can change
        return run (
                () -> { 
                    armMotor.set(0);
                }
        ).withName("stop");
    }
}
    
