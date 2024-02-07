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
import frc.robot.Constants.SpeakerShooterConstants;

public class SpeakerShooterSubsystem extends SubsystemBase{
    public TalonFX speakerShooterMotor = new TalonFX(SpeakerShooterConstants.kSpeakerShooterMotorId);
    public TalonFX speakerShooterMotor2 = new TalonFX(SpeakerShooterConstants.kSpeakerShooterMotor2Id);

    public SpeakerShooterSubsystem() {
        speakerShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        speakerShooterMotor2.setNeutralMode(NeutralModeValue.Coast);
        setDefaultCommand(stop());

        SmartDashboard.setDefaultNumber("Shooter Speed", SpeakerShooterConstants.kSpeakerMotorSpeed);
    }

    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null){
            SmartDashboard.putString("Speakershoot Command", currentCommand.getName());
        } else {
            SmartDashboard.putString("Speakershoot Command", "");
        }
    }

    public Command shootSpeaker() { //TODO: can change
        return run (
                () -> {speakerShooterMotor.set(SmartDashboard.getNumber("Shooter Speed", SpeakerShooterConstants.kSpeakerMotorSpeed)); 
                    speakerShooterMotor2.set(SmartDashboard.getNumber("Shooter Speed", SpeakerShooterConstants.kSpeakerMotorSpeed));
                }
        ).withName("shootSpeaker");
    }   

    public Command stop() { //TODO: can change
        return run (
                () -> {speakerShooterMotor.set(0); 
                    speakerShooterMotor2.set(0);
                }
        ).withName("stop");
    }
}
