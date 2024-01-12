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
    private final TalonFX intakeMotor1 = new TalonFX(IntakeConstants.kintakeMotor1Id);
    private final TalonFX intakeMotor2 = new TalonFX(IntakeConstants.kintakeMotor2Id);

    public IntakeSubsystem() {
        intakeMotor1.setNeutralMode(NeutralModeValue.Coast);
        intakeMotor2.setNeutralMode(NeutralModeValue.Coast);
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

    public Command intakeAmp() { //Subject to change due to motor shenanigans
        return run (
                () -> {intakeMotor1.set(1);
                    intakeMotor2.set(1);
                }
        ).withName("intakeAmp");
    }

    public Command intakeSpeaker()  { //Subject to change due to motor shenanigans
                return run (
                () -> {intakeMotor1.set(1);
                    intakeMotor2.set(1);
                }
        ).withName("intakeSpeaker");
    }
    public Command stop() {
        return run(
            () -> {intakeMotor1.set(0);
                intakeMotor2.set(0);
                }
        ).withName("stop");
    }
}



