package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;
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
    int counter = 0;
    @Override
    public void initialize() {
        Color8Bit old = m_DriveStationSubsystem.setColor;
        m_DriveStationSubsystem.setColor = new Color8Bit(0,0,0);
        m_DriveStationSubsystem.setLights();
        m_DriveStationSubsystem.setColor = old;
    }

    @Override
    public void execute() {
        counter++;
        counter %= 50;
        if (counter != 0) {
            return;
        }
        System.out.println("a");
        if (max >= LightLEDConstants.kStreakLength){
            min++;
        }
        if (max + 1 < m_DriveStationSubsystem.m_ledBuffer.getLength()){
            max++;
        }
        if (min > 0) {
            m_DriveStationSubsystem.setLight(min-1, new Color8Bit(0,0,0));
        }
        for (int i = min; i <= max; i++){
            System.out.println("b");
            m_DriveStationSubsystem.setLight(i, m_DriveStationSubsystem.setColor);
        } 
        m_DriveStationSubsystem.updateLight();
        System.out.println("c");
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
