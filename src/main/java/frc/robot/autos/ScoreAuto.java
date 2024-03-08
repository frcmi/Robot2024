package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Autos;

public class ScoreAuto extends SequentialCommandGroup {
    public ScoreAuto(String[] notes, RobotContainer robot) {
        addCommands(Autos.shoot(robot.swerveSubsystem, robot.intakeSubsystem));

        for (var path : notes) {
            var pathAuto = Autos.pathplannerPath(path);
            var shoot = Autos.shoot(robot.swerveSubsystem, robot.intakeSubsystem);

            addCommands(pathAuto, shoot);
        }
    }
}
