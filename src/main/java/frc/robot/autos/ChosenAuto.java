package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoChooser;

public class ChosenAuto extends Command {
    public ChosenAuto(AutoChooser chooser) {
        autoChooser = chooser;
        currentAuto = null;
    }

    @Override
    public void execute() {
        // if (currentAuto == null) {
        //     currentAuto = autoChooser.getCommand();
        // }

        // currentAuto.execute();
    }

    @Override
    public void end(boolean interrupted) {
        currentAuto = null;
    }

    @Override
    public boolean isFinished() {
        if (currentAuto == null) {
            return false;
        }

        return currentAuto.isFinished();
    }

    private Command currentAuto;
    private final AutoChooser autoChooser;
}
