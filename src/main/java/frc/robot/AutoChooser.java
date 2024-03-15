package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.ScoreAuto;
import frc.robot.commands.Autos;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoChooser {
    public static final int TARGET_COUNT = 3;
    public static final String[] NOTES = new String[8];

    private final ShuffleboardTab shuffleboard;
    private final SendableChooser<Strategy> strategyChooser;
    private final ArrayList<SendableChooser<Optional<String>>> targetChoosers;
    private final RobotContainer robotContainer;

    static {
        NOTES[0] = "Note 1";
        NOTES[1] = "Note 2";
        NOTES[2] = "Note 3";
        NOTES[3] = "Note 4";
        NOTES[4] = "Note 5";
        NOTES[5] = "Note 6";
        NOTES[6] = "Note 7";
        NOTES[7] = "Note 8";
    }

    public enum Strategy {
        TESTING,
        TRAVEL,
        PP_TRAVEL,
        TRAVEL_ANGLED,
        PP_AMP,
        PP_SCORE,
        SCORE_THEN_TRAVEL_ANGLED,
        SCORE_THEN_TRAVEL,
        PP_SCORE_THEN_TRAVEL,
        PP_SCORE_AND_RELOAD,
        SHOOT,
        NONE,
    }

    public AutoChooser(RobotContainer robot) {
        shuffleboard = Shuffleboard.getTab("Autos");
        targetChoosers = new ArrayList<>(TARGET_COUNT);
        robotContainer = robot;

        for (int i = 0; i < TARGET_COUNT; i++) {
            var chooser = new SendableChooser<Optional<String>>();
            chooser.setDefaultOption("No Target", Optional.empty());

            for (String auto : NOTES) {
                chooser.addOption(auto, Optional.of(auto));
            }

            shuffleboard.add("Target " + (i + 1), chooser);
            targetChoosers.add(chooser);
        }

        strategyChooser = new SendableChooser<>();
        shuffleboard.add("Strategy", strategyChooser);

        for (Strategy strategy : Strategy.values()) {
            String name = strategy.toString();
            if (strategy == Strategy.TRAVEL) {
                strategyChooser.setDefaultOption(name, strategy);
            } else {
                strategyChooser.addOption(name, strategy);
            }
        }
    }

    /**
     * Gets the currently selected strategy.
     */
    public Strategy getStrategy() {
        return strategyChooser.getSelected();
    }

    /**
     * Retrieves Shuffleboard-selected target positions.
     * @return The positions of the selected targets.
     */
    public String[] getNotes() {
        var targets = new ArrayList<String>();
        for (int i = 0; i < TARGET_COUNT; i++) {
            var chooser = targetChoosers.get(i);
            var value = chooser.getSelected();

            value.ifPresent(targets::add);
        }

        var result = new String[TARGET_COUNT];
        targets.toArray(result);

        return result;
    }

    public Command getCommand() {
        SwerveSubsystem swerve = robotContainer.swerveSubsystem;
        IntakeSubsystem intake = robotContainer.intakeSubsystem;

        // double travelAngle = -0.7;
        // double subwooferToSpeakerAngle = -1.5 * Math.PI;
        Translation2d angledTravelDirection = new Translation2d(0.813733, -0.581238 * 0.5);
        // double travelAngle = -0.7;
        // double subwooferToSpeakerAngle = -1.5 * Math.PI;
        // Translation2d angledTravelDirection = new Translation2d(1.2, new Rotation2d(subwooferToSpeakerAngle + travelAngle));


        Strategy strategy = getStrategy();

        Command pathplannerTravel = Autos.pathplannerAuto("Travel");
        Command travel =
                new RepeatCommand((new InstantCommand(() -> {swerve.drive(
                    new Translation2d(2 * 6, -0.5 * 6), 
                    0, 
                    false, 
                    false
                );}, swerve))).withTimeout(2.5);
        Command travelAngled =
               new RepeatCommand((new InstantCommand(() -> {swerve.drive(
                    angledTravelDirection.times(0.5), 
                    0, 
                    false, 
                    false
                );}, swerve))).withTimeout(2.5);
        Command shoot = new WaitCommand(2).andThen(intake.shoot()).andThen(new WaitCommand(0.5));

        switch (strategy) {
            case TESTING -> {
                return Autos.pathplannerAuto("Far 1");
            }
            case SHOOT -> {
                return shoot;
            }
            case TRAVEL -> {
                return travel;
            }
            case PP_TRAVEL -> {
                return pathplannerTravel;
            }
            case TRAVEL_ANGLED -> {
                return travelAngled;
            }
            case PP_AMP -> {
                return Autos.pathplannerAuto("Amp")
                    .andThen(robotContainer.ampArmSubsystem.raiseToAmp().withTimeout(1))
                    .andThen(robotContainer.ampShooterSubsystem.shootAmp().withTimeout(1))
                    .andThen(robotContainer.ampArmSubsystem.lowerArm());
            }
            case PP_SCORE -> {
                return new ScoreAuto(null, robotContainer);
            }
            case SCORE_THEN_TRAVEL -> {
                return shoot.andThen(travel);
            }
            case PP_SCORE_THEN_TRAVEL -> {
                return shoot.andThen(pathplannerTravel);
            }
            case SCORE_THEN_TRAVEL_ANGLED -> {
                return shoot.andThen(travelAngled);
            }
            case PP_SCORE_AND_RELOAD -> {
                var notes = getNotes();
                if (notes.length == 0) {
                    break;
                }

                return new ScoreAuto(notes, robotContainer);
            }
            case NONE -> {
                System.out.println("Auto is disabled");
                break;
            }
            default -> {
                System.out.println("Unhandled strategy - disabling auto");
                break;
            }
        }

        return Commands.runOnce(() -> {});
    };

}
