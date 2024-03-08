package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.ScoreAuto;

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
//        BACKUP,
        // INTERFERE,
        SCORE,
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
            if (strategy == Strategy.SCORE) {
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
            case SCORE -> {
                String[] notes = getNotes();
                if (notes.length == 0) {
                    return Commands.runOnce(() -> {});
                }

                return new ScoreAuto(notes, robotContainer);
            }
//            case BACKUP -> {
//                // TODO: impl
//                break;
//            }
            case NONE -> {
                return Commands.run(() -> {
                });
            }
        }

        System.err.println("Unhandled state " + strategy + " defaulting to nothing...");
        return Commands.run(() -> {});
    };

}
