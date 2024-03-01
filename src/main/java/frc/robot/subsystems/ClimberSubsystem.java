package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax leftClimberMotor = new CANSparkMax(ClimberConstants.kLeftClimberId, MotorType.kBrushless);
    private final CANSparkMax rightClimberMotor = new CANSparkMax(ClimberConstants.kRightClimberId, MotorType.kBrushless);
    // defines the two motors being used for the climbers


    public ClimberSubsystem() {
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);

        setDefaultCommand(stop());
    }


    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null){
            SmartDashboard.putString("Intake Command", currentCommand.getName());
        } else {
            SmartDashboard.putString("Intake Command", "");
        }

        SmartDashboard.putNumber("Climber Current", leftClimberMotor.getOutputCurrent());
    }

    public Command up() { //Subject to change due to motor shenanigans
        return run (
                () -> {leftClimberMotor.set(ClimberConstants.kClimberUp);
                    rightClimberMotor.set(ClimberConstants.kClimberUp);
                }
        ).until(() -> leftClimberMotor.getOutputCurrent() > 100 || leftClimberMotor.getOutputCurrent() > 100).withName("up");
    }

    public Command down() { //Subject to change due to motor shenanigans
        return run (
                () -> {leftClimberMotor.set(ClimberConstants.kClimberDown);
                    rightClimberMotor.set(ClimberConstants.kClimberDown);
                }
        ).withName("down");
    }
    
    public Command stop() {
        return run(
            () -> {leftClimberMotor.set(0);
                rightClimberMotor.set(0);
                }
        ).withName("stop");
    }
}



