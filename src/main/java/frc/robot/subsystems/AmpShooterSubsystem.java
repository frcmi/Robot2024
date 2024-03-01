package frc.robot.subsystems;

import java.util.Map;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.Constants.AmpShooterConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class AmpShooterSubsystem extends SubsystemBase{
    private final CANSparkMax shootMotor = new CANSparkMax(AmpShooterConstants.kShootMotor, MotorType.kBrushless);
    private final AmpArmSubsystem ampArmSubsystem;
 
    public AmpShooterSubsystem(AmpArmSubsystem ampArm) {
        ampArmSubsystem = ampArm;
        shootMotor.setInverted(true);
        setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null){
            SmartDashboard.putString("AmpShooter Command", currentCommand.getName());
        } else {
            SmartDashboard.putString("AmpShooter Command", "");
        }
    }

    public Command shootAmp() { //TODO: can change
        return run (
                () -> {shootMotor.set(-AmpShooterConstants.kAmpMotorSpeed); // Keep this motor negative
                }
        ).withName("shootAmp");
    }

    public Command stop() { //TODO: can change
        return runOnce (
                () -> {
                    shootMotor.set(0);
                }
        )
        .withName("stop");
    }
}
    
