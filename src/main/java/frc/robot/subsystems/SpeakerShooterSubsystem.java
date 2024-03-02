package frc.robot.subsystems;

import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.XboxController;
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
    public BooleanSupplier overrideBeamBreak;
    public SpeakerShooterSubsystem(IntakeSubsystem intake, BooleanSupplier overrideBeambreak) {
        this.intakeSubsystem = intake;
        this.overrideBeamBreak = overrideBeambreak;
        speakerShooterMotor.setNeutralMode(NeutralModeValue.Brake);

        SmartDashboard.setDefaultNumber("Shooter Speed", SpeakerShooterConstants.kSpeakerMotorSpeed);
        setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            // SmartDashboard.putString("Speakershoot Command", currentCommand.getName());
        } else {
            // SmartDashboard.putString("Speakershoot Command", "");
        }

        // if (!intakeSubsystem.beambreak.get() || this.overrideBeamBreak.getAsBoolean()) {
        //     speakerShooterMotor.set(SmartDashboard.getNumber("Shooter Speed", SpeakerShooterConstants.kSpeakerMotorSpeed));
        // } else {
        //     speakerShooterMotor.set(0);
        // }
    }

     public Command shoot() {
        return run(() -> {speakerShooterMotor.set(SpeakerShooterConstants.kSpeakerMotorSpeed);}).withName("stop");
    }

    public Command stop() {
        return run(() -> {speakerShooterMotor.set(0);}).withName("stop");
    }
}
