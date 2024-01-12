package frc.robot.simulation;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveConstants;

public class SimulatedSwerveModule extends SwerveModule {
    FlywheelSim steerMotor = new FlywheelSim(DCMotor.getFalcon500(1), SwerveConstants.kSteerMotorGearRatio, 0.004096955);
    FlywheelSim driveMotor = new FlywheelSim(DCMotor.getFalcon500(1), /* i have no idea but i think we should google it */);
    public SimulatedSwerveModule(int moduleNumber, Rotation2d angleOffset, SwerveModuleConstants moduleConstants) {
        super(moduleNumber, angleOffset, moduleConstants);

    }
    
}
