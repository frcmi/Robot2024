package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that drives the robot with controller inputs
 */
public class TeleopSwerve extends Command {    
    private SwerveSubsystem s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(Math.pow(translationSup.getAsDouble(), 3), Constants.OperatorConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(Math.pow(strafeSup.getAsDouble(), 3), Constants.OperatorConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(Math.pow(rotationSup.getAsDouble(), 3), Constants.OperatorConstants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(RobotBase.isReal() ? SwerveConstants.maxSpeed : SimulationConstants.kSimulationMaxSpeed), 
            rotationVal * (RobotBase.isReal() ? SwerveConstants.maxAngularVelocity : SimulationConstants.kSimulationMaxRotationSpeed), 
            !robotCentricSup.getAsBoolean(),
            true
        );
    }
}