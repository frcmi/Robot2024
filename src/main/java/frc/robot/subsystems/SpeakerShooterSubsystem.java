package frc.robot.subsystems;

import java.util.Map;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SpeakerShooterConstants;

public class SpeakerShooterSubsystem extends SubsystemBase {
    public TalonFX speakerShooterMotor = new TalonFX(SpeakerShooterConstants.kSpeakerShooterMotorId);
    public IntakeSubsystem intakeSubsystem;
    public SpeakerShooterSubsystem(IntakeSubsystem intake) {
        this.intakeSubsystem = intake;
        speakerShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        setDefaultCommand(stop());

        SmartDashboard.setDefaultNumber("Shooter Speed", SpeakerShooterConstants.kSpeakerMotorSpeed);
    }

    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            SmartDashboard.putString("Speakershoot Command", currentCommand.getName());
        } else {
            SmartDashboard.putString("Speakershoot Command", "");
        }

        if (!intakeSubsystem.beambreak.get()) {
            speakerShooterMotor.set(SmartDashboard.getNumber("Shooter Speed", SpeakerShooterConstants.kSpeakerMotorSpeed));
        } else {
            speakerShooterMotor.set(0);
        }
    }

    public Command shootSpeaker() { //TODO: can change
        return new ParallelCommandGroup(run (
                () -> {speakerShooterMotor.set(SmartDashboard.getNumber("Shooter Speed", SpeakerShooterConstants.kSpeakerMotorSpeed)); 
                }
        ), new WaitCommand(1).andThen(intakeSubsystem.intakeSpeakerNoBeamBreak())).withName("shootSpeaker");
    }   

    // public Command shootSpeakerAndStop() {
    //     return shootSpeaker().andThen(Commands.waitSeconds(SpeakerShooterConstants.kSpeakerShootDuration))
    //             .andThen(stop());
    // }

    // public Command stop() { // TODO: can change
    //     return run(
    //             () -> {
    //                 speakerShooterMotor.set(0);
    //             }).withName("stop");
    // }
}
