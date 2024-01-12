package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX shooterMotor = new TalonFX(ShooterConstants.kshooterMotorId);

    public ShooterSubsystem() {
        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        var currentCommand = this.CurrentCommand();`
        if (currentCommand != null){
            SmartDashboard.putString("Outtake Command", currentCommand.getName());
        } else {
            SmartDashboard.putString("Outtake Command", "");
        }
    }

    public Command shoot() {
        return run (
                () -> {shooterMotor.set(1); //TODO: can change
                }
        ).withName("intake");
    }

    public Command stop() {
        return run (
                () -> {shooterMotor.set(0); //TODO: can change
                }
        ).withName("stop");
    }
}
    
