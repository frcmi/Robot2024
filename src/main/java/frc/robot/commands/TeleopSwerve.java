package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private SlewRateLimiter translationLimit = null;
    private SlewRateLimiter strafeLimit = null;
    private final BooleanSupplier slewEnableSupplier;

    public TeleopSwerve(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier slewEnableSupplier) {
        this.slewEnableSupplier = slewEnableSupplier;
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        boolean slewEnabled = slewEnableSupplier.getAsBoolean();
        int pow = slewEnabled ? 1 : 3;
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(Math.pow(translationSup.getAsDouble(), pow), Constants.OperatorConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(Math.pow(strafeSup.getAsDouble(), pow), Constants.OperatorConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(Math.pow(rotationSup.getAsDouble(), pow), Constants.OperatorConstants.stickDeadband);
        
        if (slewEnabled) {
            if (strafeLimit == null) {
                strafeLimit = new SlewRateLimiter(0.8);
                translationLimit = new SlewRateLimiter(0.8);
            }

            translationVal = translationVal == 0 ? 0 : 1 * translationLimit.calculate(translationVal);
            strafeVal = strafeVal == 0 ? 0 : 1 * strafeLimit.calculate(strafeVal);
        } else {
            if (strafeLimit != null) {
                strafeLimit = null;
                translationLimit = null;
            }
        }

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            translationVal *= -1;
            strafeVal *= -1;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(RobotBase.isReal() ? Constants.SwerveConstants.maxSpeed : Constants.SimulationConstants.kSimulationMaxSpeed),
            rotationVal * (RobotBase.isReal() ? Constants.SwerveConstants.maxAngularVelocity : Constants.SimulationConstants.kSimulationMaxRotationSpeed),
            !robotCentricSup.getAsBoolean(),
            false
        );
    }
}