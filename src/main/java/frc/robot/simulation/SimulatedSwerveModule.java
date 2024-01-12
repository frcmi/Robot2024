package frc.robot.simulation;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.SwerveModule;

public class SimulatedSwerveModule extends SwerveModule {

    public SimulatedSwerveModule(int moduleNumber, Rotation2d angleOffset, SwerveModuleConstants moduleConstants) {
        super(moduleNumber, angleOffset, moduleConstants);
        
    }
    
}
