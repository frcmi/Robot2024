package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LightLEDConstants;
import frc.robot.subsystems.DriveStationSubsystem;

public class SetTrailLights extends Command {
    DriveStationSubsystem m_DriveStationSubsystem;

    public SetTrailLights(DriveStationSubsystem DriveStationSubsystem) {
        m_DriveStationSubsystem = DriveStationSubsystem;
    }

    int min = 0;
    int max = 0;

    @Override
    public void execute() {
        if (max >= LightLEDConstants.kStreakLength){
            min++;
        }
        if (max + 1 < m_DriveStationSubsystem.m_ledBuffer.getLength()){
            max++;
        }
        for (int i = min; i <= max; i++){
            m_DriveStationSubsystem.setLight(i, m_DriveStationSubsystem.setColor);
        } 
        m_DriveStationSubsystem.updateLight();
    } 

    @Override
    public boolean isFinished() {
        if (min >= m_DriveStationSubsystem.m_ledBuffer.getLength() && max >= m_DriveStationSubsystem.m_ledBuffer.getLength()){
            return true;
        } else {
            return false;
        }
    }
    
}
