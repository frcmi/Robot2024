package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.lib.ultralogger.UltraTempLog;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    // defines the two motors being used for the climbers
    private final CANSparkMax leftClimberMotor = new CANSparkMax(ClimberConstants.kLeftClimberId, MotorType.kBrushless);
    private final CANSparkMax rightClimberMotor = new CANSparkMax(ClimberConstants.kRightClimberId, MotorType.kBrushless);

    private final UltraDoubleLog leftCurrentPublisher = new UltraDoubleLog("Climber/Left Motor Current");
    private final UltraDoubleLog rightCurrentPublisher = new UltraDoubleLog("Climber/Right Motor Current");
    private final UltraTempLog leftTemperaturePublisher = new UltraTempLog("Climber/Left Motor Temperature", leftClimberMotor::getMotorTemperature);
    private final UltraTempLog rightTemperaturePublisher = new UltraTempLog("Climber/Right Motor Temperature", rightClimberMotor::getMotorTemperature);

    public ClimberSubsystem() {
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);

        setDefaultCommand(stop());
    }


    @Override
    public void periodic() {
        leftCurrentPublisher.update(leftClimberMotor.getOutputCurrent());
        rightCurrentPublisher.update(rightClimberMotor.getOutputCurrent());
        leftTemperaturePublisher.update();
        rightTemperaturePublisher.update();
    }

    public Command up() { //Subject to change due to motor shenanigans
        return run (
                () -> {leftClimberMotor.set(ClimberConstants.kClimberUp);
                    rightClimberMotor.set(ClimberConstants.kClimberUp);
                }
        ).until(() -> leftClimberMotor.getOutputCurrent() > 35 || leftClimberMotor.getOutputCurrent() > 35).withName("up");
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



