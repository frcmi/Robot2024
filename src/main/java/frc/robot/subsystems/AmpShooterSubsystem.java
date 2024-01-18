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
import frc.robot.Constants.AmpShooterConstants;

public class AmpShooterSubsystem extends SubsystemBase{
    private TalonFX ampShooterMotor = new TalonFX(AmpShooterConstants.kampShooterMotorId);
    private TalonFX ampAxisMotor1 = new TalonFX(AmpShooterConstants.kampAxisMotor1Id);
    private TalonFX ampAxisMotor2 = new TalonFX(AmpShooterConstants.kampAxisMotor2Id);

    public AmpShooterSubsystem() {
        ampShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        ampAxisMotor1.setNeutralMode(NeutralModeValue.Coast);
        ampAxisMotor2.setNeutralMode(NeutralModeValue.Coast);

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
                () -> {ampAxisMotor1.set(1); 
                    ampAxisMotor2.set(1);
                }
        ).withName("shootAmp");
    }

    public Command stop() { //TODO: can change
        return run (
                () -> {ampShooterMotor.set(0);
                }
        ).withName("stop");
    }
}
    
