package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.AmpShooterConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
        private final CANSparkMax climbMotorL = new CANSparkMax(ClimberConstants.kClimberMotorL, MotorType.kBrushless);
        private final CANSparkMax climbMotorR = new CANSparkMax(ClimberConstants.kClimberMotorR, MotorType.kBrushless);

    public ClimberSubsystem() {
        climbMotorL.setIdleMode(IdleMode.kBrake);
        climbMotorR.setIdleMode(IdleMode.kBrake);
        setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null){
            SmartDashboard.putString("AmpShooter Command", currentCommand.getName());
        } else {
            SmartDashboard.putString("AmpShooter Command", "");
        }
    }
    public Command descend() { //just in case (don't insult the name)
        return run (
            () -> {climbMotorL.set(-ClimberConstants.kClimbSpeed);
                climbMotorR.set(-ClimberConstants.kClimbSpeed);
            }
        ).withName("Descend");
    }

    public Command climb() {
        return run (
        () -> {climbMotorL.set(ClimberConstants.kClimbSpeed);
            climbMotorR.set(ClimberConstants.kClimbSpeed);
        }
).withName("Climb");
}

public Command stop() {
    return run (
    () -> {climbMotorL.set(0);
        climbMotorR.set(0);
    }
).withName("Stop");
}

}
