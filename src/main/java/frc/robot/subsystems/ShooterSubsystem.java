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

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX motor3 = new TalonFX(ShooterConstants.kmotor3Id);

    public ShooterSubsystem() {
        motor3.setNeutralMode(NeutralModeValue.Coast);
        setDefaultCommand(stop());
    }

    public Command shoot() {
        return run (
                () -> {motor3.set(1);
                }
        ).withName("intake");
    }

    public Command stop() {
        return run (
                () -> {motor3.set(0);
                }
        ).withName("stop");
    }
}
    
