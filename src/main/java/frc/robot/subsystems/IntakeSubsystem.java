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
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX motor1 = new TalonFX(IntakeConstants.kmotor1Id);
    private final TalonFX motor2 = new TalonFX(IntakeConstants.kmotor2Id);

    public IntakeSubsystem() {
        motor1.setNeutralMode(NeutralModeValue.Coast);
        motor2.setNeutralMode(NeutralModeValue.Coast);
        setDefaultCommand(stop());
    }


    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null){
            SmartDashboard.putString("Intake Command", currentCommand.getName());
        } else {
            SmartDashboard.putString("Intake Command", "");
        }
    }

    public Command intake() {
        return run (
                () -> {motor1.set(1);
                    motor2.set(1);
                }
        ).withName("intake");
    }
    public Command stop() {
        return run(
            () -> {motor1.set(0);
                motor2.set(0);
                }
        ).withName("stop");
    }
}



