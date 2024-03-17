package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ultralogger.UltraBooleanLog;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.lib.ultralogger.UltraTempLog;
import frc.robot.Constants;
import frc.robot.Constants.SpeakerShooterConstants;

public class SpeakerShooterSubsystem extends SubsystemBase {
    public TalonFX speakerShooterMotor = new TalonFX(SpeakerShooterConstants.kSpeakerShooterMotorId);

    public DigitalInput beambreak = new DigitalInput(Constants.IntakeConstants.kSpeakerBeamBreakPort);
    public BooleanSupplier manualRev;
    private final UltraBooleanLog beambreakPublisher = new UltraBooleanLog("Speaker Shooter/Beambreak");
    private final UltraTempLog temperaturePublisher = new UltraTempLog("Speaker Shooter/Motor Temperature", speakerShooterMotor.getDeviceTemp().asSupplier());

    public SpeakerShooterSubsystem(BooleanSupplier manualRev) {
        this.manualRev = manualRev;
        speakerShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        boolean beamNotBroken = beambreak.get();

        temperaturePublisher.update();
        beambreakPublisher.update(beamNotBroken);
        // TODO: remove this once UltraLog supports always networked values
        // This is needed so driver can see if note is actually in the speaker shooter
        SmartDashboard.putBoolean("Speaker Beam Break", beamNotBroken);

        if (!beamNotBroken || manualRev.getAsBoolean()) {
            shoot().schedule();
        } else {
            stop().schedule();
        }
    }

    public Command shoot() {
        return run(() -> {
         speakerShooterMotor.set(SpeakerShooterConstants.kSpeakerMotorSpeed);
        }).withName("shoot");
    }

     public Command stop() {
         return run(() -> {
             speakerShooterMotor.set(0);
         }).withName("stop");
     }
}
