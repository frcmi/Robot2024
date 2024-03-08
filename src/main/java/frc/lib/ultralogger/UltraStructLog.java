package frc.lib.ultralogger;

import frc.robot.Constants.TelemetryConstants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.Optional;

public class UltraStructLog<T> implements UltraLogEntry<T> {
    public String logName;
    private Optional<StructPublisher<T>> ntPublisher = Optional.empty();
    private final StructLogEntry<T> datalogPublisher;
    private double lastCheckedTimestamp = System.currentTimeMillis();

    private final Struct<T> struct;

    public UltraStructLog(String name, Struct<T> struct) {
        this.struct = struct;
        this.logName = TelemetryConstants.tabPrefix + name;
        checkFMS(false);
        this.datalogPublisher = StructLogEntry.create(DataLogManager.getLog(), logName, struct);
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
            this.ntPublisher = Optional.of(NetworkTableInstance.getDefault().getStructTopic(logName, struct).publish());
        }
    }

    public void update(T item) {
        if (item == null) {return;}
        this.datalogPublisher.append(item);
        if (this.ntPublisher.isPresent()) {
            ntPublisher.get().set(item);
            checkFMS(true);
        }
    }
}
