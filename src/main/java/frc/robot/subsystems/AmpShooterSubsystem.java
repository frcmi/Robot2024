package frc.robot.subsystems;

import java.util.Map;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AmpShooterConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class AmpShooterSubsystem extends SubsystemBase{
    // private TalonFX ampShooterMotor = new TalonFX(AmpShooterConstants.kampShooterMotorId);
    private final CANSparkMax upperMotor = new CANSparkMax(AmpShooterConstants.kUpperAmpMotorId, MotorType.kBrushless);
    private final CANSparkMax lowerMotor = new CANSparkMax(AmpShooterConstants.kLowerAmpMotorId, MotorType.kBrushless);

    public AmpShooterSubsystem() {
        upperMotor.setInverted(true);
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
                () -> {upperMotor.set(-AmpShooterConstants.kAmpMotorSpeed); // Keep this motor negative
                    lowerMotor.set(AmpShooterConstants.kAmpMotorSpeed);
                }
        ).withName("shootAmp");
    }

    public Command stop() { //TODO: can change
        return run (
                () -> { //ampShooterMotor.set(0);
                    upperMotor.set(0);
                    lowerMotor.set(0);
                }
        ).withName("stop");
    }
}
    
