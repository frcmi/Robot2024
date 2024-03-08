package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.ScoreAuto;
import frc.robot.commands.Autos;

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
        TRAVEL,
        SCORE,
        SCORE_THEN_TRAVEL,
        SCORE_AND_RELOAD,
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
        Strategy strategy = getStrategy();
        switch (strategy) {
            case TRAVEL -> {
                return Autos.pathplannerPath("Travel");
            }
            case SCORE -> {
                return new ScoreAuto(null, robotContainer);
            }
            case SCORE_THEN_TRAVEL -> {
                Command score = new ScoreAuto(null, robotContainer);
                Command travel = Autos.pathplannerPath("Travel");
                
                return score.andThen(travel);
            }
            case SCORE_AND_RELOAD -> {
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

        return Commands.waitSeconds(0);
    };

}
