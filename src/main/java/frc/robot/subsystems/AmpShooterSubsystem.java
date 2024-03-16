package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ultralogger.UltraBooleanLog;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.lib.ultralogger.UltraTempLog;
import frc.robot.Constants.AmpShooterConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class AmpShooterSubsystem extends SubsystemBase{
    private final CANSparkMax shootMotor = new CANSparkMax(AmpShooterConstants.kShootMotor, MotorType.kBrushless);
    private final AmpArmSubsystem ampArmSubsystem;
    private final LEDSubsystem ledSubsystem;

    private final DigitalInput beambreak = new DigitalInput(AmpShooterConstants.kAmpBeamBrakeId);

    private final UltraDoubleLog currentPublisher = new UltraDoubleLog("Amp Shooter/Motor Current");
    private final UltraBooleanLog beambreakPublisher = new UltraBooleanLog("Amp Shooter/Beambreak");
    private final UltraTempLog temperaturePublisher = new UltraTempLog("Amp Shooter/Motor Temperature", shootMotor::getMotorTemperature);

    public AmpShooterSubsystem(AmpArmSubsystem ampArm, LEDSubsystem leds) {
        ampArmSubsystem = ampArm;
        ledSubsystem = leds;
        shootMotor.setInverted(true);
        setDefaultCommand(stop());
    }

    boolean beamClearPrevious = true;
    @Override
    public void periodic() {
        boolean beamClear = beambreak.get();

        currentPublisher.update(shootMotor.getOutputCurrent());
        beambreakPublisher.update(beamClear);
        temperaturePublisher.update();

        // TODO: remove this once UltraLog supports always networked values
        // This is needed to driver can see if note is actually in the amp shooter
        SmartDashboard.putBoolean("Amp Beam Break", beamClear);

        if (!beambreak.get()) {
            ledSubsystem.readyToAmp().schedule();
        }
        if (!beamClearPrevious && beamClear) {
            ledSubsystem.ledOff().schedule();
        }
        beamClearPrevious = beamClear;

        SmartDashboard.putNumber("Motor Output", shootMotor.getAppliedOutput());
    }

    public Command shootAmp() { //TODO: can change
        return run (
            () -> {
                shootMotor.set(-AmpShooterConstants.kAmpMotorSpeed); // Keep this motor negative
            }
        ).withName("shootAmp");
    }

    public Command intakeAmp() {
        return run (
            () -> {
                shootMotor.set(0.4); // Keep this motor negative
            }
        )
                .until(() -> !beambreak.get())
                .andThen(Commands.waitSeconds(0.09))
                .andThen(stop())
                .withName("intakeAmp");
    }

    public Command stop() { //TODO: can change
        return runOnce (
            () -> {
                shootMotor.set(0);
            }
        ).withName("stop");
    }
}
    
