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

public class AmpShooterSubsystem extends SubsystemBase{
    private TalonFX speakerShooterMotor = new TalonFX(ShooterConstants.kspeakerShooterMotorId);
    private TalonFX ampShooterMotor = new TalonFX(ShooterConstants.kampShooterMotorId);
    private TalonFX ampAxisMotor1 = new TalonFX(ShooterConstants.kampAxisMotor1Id);
    private TalonFX ampAxisMotor2 = new TalonFX(ShooterConstants.kampAxisMotor2Id);

    public AmpShooterSubsystem() {
        speakerShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        ampShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        ampAxisMotor1.setNeutralMode(NeutralModeValue.Coast);
        ampAxisMotor2.setNeutralMode(NeutralModeValue.Coast);

        setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null){
            SmartDashboard.putString("Outtake Command", currentCommand.getName());
        } else {
            SmartDashboard.putString("Outtake Command", "");
        }
    }

    public Command shootAmp() { //TODO: can change
        return run (
                () -> {ampAxisMotor1.set(1); 
                    ampAxisMotor2.set(1);
                }
        ).withName("shootAmp");
    }

    public Command shootSpeaker() { //TODO: can change
        return run (
                () -> {speakerShooterMotor.set(1); 
                }
        ).withName("shootSpeaker");
    }    

    public Command stop() { //TODO: can change
        return run (
                () -> {speakerShooterMotor.set(0); 
                    ampShooterMotor.set(0);
                }
        ).withName("stop");
    }
}
    
