package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoChooser;

public class AutoChooserCommand extends Command {
    public AutoChooserCommand(AutoChooser chooser) {
        autoChooser = chooser;
    }

    @Override
    public void execute() {
        // this function is run repeatedly, but theres a flag in AutoChooser
        autoChooser.startChooser();
    }

    @Override
    public void end(boolean interrupted) {
        autoChooser.stopChooser();
    }

    private AutoChooser autoChooser;
}
