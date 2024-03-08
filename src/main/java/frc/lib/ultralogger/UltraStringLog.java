package frc.lib.ultralogger;

import frc.robot.Constants.TelemetryConstants;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.Optional;

public class UltraStringLog implements UltraLogEntry<String> {
    public String logName;
    private Optional<StringPublisher> ntPublisher = Optional.empty();
    private Optional<StringLogEntry> datalogPublisher = Optional.empty();
    private double lastCheckedTimestamp = System.currentTimeMillis();

    public UltraStringLog(String name) {
        this.logName = TelemetryConstants.tabPrefix + name;

        checkNTFMS(false);
        checkDLFMS();
    }

    private void checkDLFMS() {
        if (UltraLogEntry.disableDatalog()) {
            return;
        }

        this.datalogPublisher = Optional.of(new StringLogEntry(DataLogManager.getLog(), logName));
    }

    private void checkNTFMS(boolean doTimestamp) {
        if (doTimestamp && System.currentTimeMillis() - lastCheckedTimestamp < TelemetryConstants.fmsCheckDelay) {
            return;
        }

        this.lastCheckedTimestamp = System.currentTimeMillis();

        if (UltraLogEntry.disableNetworkTableLogs()) {
            this.ntPublisher = Optional.empty();
            return;
        }

        if (this.ntPublisher.isEmpty()) {
            this.ntPublisher = Optional.of(NetworkTableInstance.getDefault().getStringTopic(logName).publish());
        }
    }

    public void update(String item) {
        if (item == null) {return;}

        if (this.datalogPublisher.isPresent()) {
            this.datalogPublisher.get().append(item);
        } else {
            checkDLFMS();
        }

        if (this.ntPublisher.isPresent()) {
            ntPublisher.get().set(item);
            checkNTFMS(true);
        }
    }
}
