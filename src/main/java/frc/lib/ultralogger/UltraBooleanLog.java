package frc.lib.ultralogger;

import frc.robot.Constants.TelemetryConstants;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.Optional;

public class UltraBooleanLog implements UltraLogEntry<Boolean> {
    public String logName;
    private Optional<BooleanPublisher> ntPublisher = Optional.empty();
    private final BooleanLogEntry datalogPublisher;
    private double lastCheckedTimestamp = System.currentTimeMillis();

    public UltraBooleanLog(String name) {
        this.logName = TelemetryConstants.tabPrefix + name;
        checkFMS(false);
        this.datalogPublisher = new BooleanLogEntry(DataLogManager.getLog(), logName);
    }

    private void checkFMS(boolean doTimestamp) {
        if (doTimestamp && System.currentTimeMillis() - lastCheckedTimestamp < TelemetryConstants.fmsCheckDelay) {
            return;
        }

        this.lastCheckedTimestamp = System.currentTimeMillis();

        if (UltraLogEntry.dontNetwork()) {
            this.ntPublisher = Optional.empty();
            return;
        }

        if (this.ntPublisher.isEmpty()) {
            this.ntPublisher = Optional.of(NetworkTableInstance.getDefault().getBooleanTopic(logName).publish());
        }
    }

    public void update(Boolean item) {
        if (item == null) {return;}        this.datalogPublisher.append(item);
        if (this.ntPublisher.isPresent()) {
            ntPublisher.get().set(item);
            checkFMS(true);
        }
    }
}
