package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ultralogger.UltraBooleanLog;
import frc.robot.Constants.IntakeConstants;

import java.util.function.BooleanSupplier;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor1 = new TalonFX(IntakeConstants.kIntakeMotor1Id);
    private final TalonFX intakeMotor2 = new TalonFX(IntakeConstants.kIntakeMotor2Id);
    private final TalonFX indexerMotor = new TalonFX(IntakeConstants.kIndexerMotorId);

    private final BooleanSupplier speakerBeambreak;

    public IntakeSubsystem(BooleanSupplier speakerBeambreak) {
        this.speakerBeambreak = speakerBeambreak;
        intakeMotor1.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor2.setNeutralMode(NeutralModeValue.Brake);
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        setDefaultCommand(stop());
    }

    public Command intakeAmp() { //Subject to change due to motor shenanigans
        return run (
            () -> {
                intakeMotor1.set(IntakeConstants.kIntakeMotorSpeed);
                intakeMotor2.set(-IntakeConstants.kIntakeMotorSpeed);
                indexerMotor.set(-IntakeConstants.kIndexerSpeed);
            }
        ).withName("intakeAmp");
    }

    public Command intakeSpeaker()  { //Subject to change due to motor shenanigans
        return intakeSpeakerNoBeamBreak().until(() -> !speakerBeambreak.getAsBoolean()).andThen(stop()).withName("intakeSpeaker with beambreak");
    }

    public Command shoot()  { //Subject to change due to motor shenanigans
        return intakeSpeakerNoBeamBreak(2).until(speakerBeambreak::getAsBoolean).andThen(stop()).withName("shoot with beambreak");
    }

    public Command intakeSpeakerNoBeamBreak()  { //Subject to change due to motor shenanigans
                return intakeSpeakerNoBeamBreak(1);
    }

    public Command intakeSpeakerNoBeamBreak(double speed )  { //Subject to change due to motor shenanigans
        return run(
            () -> {
                intakeMotor1.set(IntakeConstants.kIntakeMotorSpeed * speed);
                intakeMotor2.set(-IntakeConstants.kIntakeMotorSpeed * speed);
                indexerMotor.set(IntakeConstants.kIndexerSpeed * speed);
            }
        ).withName("intakeSpeaker");
    }


    public Command extractNote() {
        return run (
            () -> {intakeMotor1.set(-IntakeConstants.kIntakeMotorSpeed);
                intakeMotor2.set(IntakeConstants.kIntakeMotorSpeed);
                indexerMotor.set(-IntakeConstants.kIndexerSpeed);

            }
        ).withName("extractNote");
    }
    
    public Command stop() {
        return run(
            () -> {
                intakeMotor1.set(0);
                intakeMotor2.set(0);
                indexerMotor.set(0);
            }
        ).withName("stop");
    }
}
