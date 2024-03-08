package frc.lib.ultralogger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TelemetryConstants;

public interface UltraLogEntry<T> {
    String logName = "";

    void update(T item);

    static boolean dontNetwork() {
        return TelemetryConstants.disableNetworkLogging || DriverStation.isFMSAttached();
    }
}
