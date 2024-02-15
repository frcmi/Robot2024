package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.Semaphore;

public class AutoChooser {
    public static final int TARGET_COUNT = 5;
    public static final HashMap<String, Pose2d> TARGETS = new HashMap<>();

    static {
        TARGETS.put("some shit on the field idk this is a placeholder", new Pose2d(7, 42, new Rotation2d(Math.PI / 2)));
    }

    public enum Strategy {
        BACKUP,
        INTERFERE,
        SCORE
    }

    public AutoChooser() {
        semaphore = new Semaphore();
        running = false;

        shuffleboard = Shuffleboard.getTab("Autos");
        targetChoosers = new ArrayList<>(TARGET_COUNT);

        for (int i = 0; i < TARGET_COUNT; i++) {
            var chooser = new SendableChooser<Pose2d>();

            shuffleboard.add("Target " + (i + 1), chooser);
            targetChoosers.add(chooser);
        }

        boolean setDefault = false;
        for (var entry : TARGETS.entrySet()) {
            for (int i = 0; i < TARGET_COUNT; i++) {
                var chooser = targetChoosers.get(i);
                if (setDefault) {
                    chooser.addOption(entry.getKey(), entry.getValue());
                } else {
                    chooser.setDefaultOption(entry.getKey(), entry.getValue());
                }
            }

            setDefault = true;
        }

        strategyChooser = new SendableChooser<>();
        shuffleboard.add("Strategy", strategyChooser);
        
        setDefault = false;
        for (var strategy : Strategy.values()) {
            var name = strategy.toString();
            if (setDefault) {
                strategyChooser.addOption(name, strategy);
            } else {
                strategyChooser.setDefaultOption(name, strategy);
                setDefault = true;
            }
        }

        strategyChooser.onChange(strategy -> {
            if (!running) {
                return;
            }
        });
    }

    /**
     * Stops the previous strategy and begins a new one.
     * @param strategy The new strategy to begin executing.
     */
    public void startStrategy(Strategy strategy) {
        // stop the previous auto
        stop();

        // make sure we wait for the semaphore to be reset before we queue a new command
        semaphore.onReady(() -> {
            var command = Commands.print("Placeholder").repeatedly();

            semaphore.addDependent();
            command.until(semaphore::checkStatus).schedule();
        });
    }

    /**
     * Stops the current strategy.
     */
    public void stop() {
        if (semaphore.hasDependents()) {
            semaphore.signal();
        }
    }

    /**
     * Starts the auto chooser process with the currently selected strategy.
     */
    public void startChooser() {
        if (running) {
            return;
        }
        
        var strategy = getStrategy();
        startStrategy(strategy);

        running = true;
    }

    /**
     * Stops the auto chooser process.
     */
    public void stopChooser() {
        if (!running) {
            return;
        }

        running = false;
        stop();
    }

    /**
     * Gets the currently selected strategy.
     */
    public Strategy getStrategy() {
        return strategyChooser.getSelected();
    }
    
    private ShuffleboardTab shuffleboard;
    private SendableChooser<Strategy> strategyChooser;
    private ArrayList<SendableChooser<Pose2d>> targetChoosers;
    private Semaphore semaphore;
    private boolean running;
}
