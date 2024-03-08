package frc.lib.ultralogger;

import frc.robot.Constants.TelemetryConstants;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.Optional;

public class UltraDoubleLog implements UltraLogEntry<Double> {
    public String logName;
    private Optional<DoublePublisher> ntPublisher = Optional.empty();
    private Optional<DoubleLogEntry> datalogPublisher = Optional.empty();
    private double lastCheckedTimestamp = System.currentTimeMillis();

    public UltraDoubleLog(String name) {
        this.logName = TelemetryConstants.tabPrefix + name;

        checkNTFMS(false);
        checkDLFMS();
    }

    private void checkDLFMS() {
        if (UltraLogEntry.disableDatalog()) {
            return;
        }

        this.datalogPublisher = Optional.of(new DoubleLogEntry(DataLogManager.getLog(), logName));
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
            this.ntPublisher = Optional.of(NetworkTableInstance.getDefault().getDoubleTopic(logName).publish());
        }
    }

    public void update(Double item) {
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
