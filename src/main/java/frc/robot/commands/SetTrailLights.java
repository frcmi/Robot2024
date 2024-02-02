package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.DriveStationSubsystem;


/**
 * Creates a trail effect on the LEDs given a {@link DriveStationSubsystem}.
 * <p>It will follow current {@link DriveStationSubsystem} LED color.
 */
public class SetTrailLights extends Command {
    private int counter = 0;
    private int frame = 0;
    private boolean done = false;
    private DriveStationSubsystem m_DriveStationSubsystem;
    private boolean forever;

    public SetTrailLights(DriveStationSubsystem DriveStationSubsystem, boolean forever) {
        m_DriveStationSubsystem = DriveStationSubsystem;
        this.forever = forever;
    }

    @Override
    public void execute() {
        counter++;
        counter %= 2;
        if (counter != 0) {
            return;
        }

        frame++;

        // No LEDs are active on first frame
        int min = -1;
        int max = -1;

        if (frame > 0) {
            // See https://www.desmos.com/calculator/fejjdhp1bc
            min = Math.max(0, frame - LEDConstants.kStreakLength);
            max = Math.min(m_DriveStationSubsystem.m_ledBuffer.getLength(), frame - 1);

            if (min == m_DriveStationSubsystem.m_ledBuffer.getLength()) {
                done = true;
            }
        }
        
        for (int i = 0; i < m_DriveStationSubsystem.m_ledBuffer.getLength(); i++) {
            if (i >= min && i <= max) {
            m_DriveStationSubsystem.setLight(i, m_DriveStationSubsystem.setColor);

            } else {
            m_DriveStationSubsystem.setLight(i, new Color8Bit());
            }
        }

    } 

    @Override
    public boolean isFinished() {
        if (done) {
            frame = 0;
            done = false;
            m_DriveStationSubsystem.setLights();
            return true && !forever;
        } else {
            return false;
        }
    }
    
}
