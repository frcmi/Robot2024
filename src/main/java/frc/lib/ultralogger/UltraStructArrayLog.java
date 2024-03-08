package frc.lib.ultralogger;

import frc.robot.Constants.TelemetryConstants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.Optional;

public class UltraStructArrayLog<T> implements UltraLogEntry<T[]> {
    public String logName;
    private Optional<StructArrayPublisher<T>> ntPublisher = Optional.empty();
    private final StructArrayLogEntry<T> datalogPublisher;
    private double lastCheckedTimestamp = System.currentTimeMillis();

    private final Struct<T> struct;

    public UltraStructArrayLog(String name, Struct<T> struct) {
        this.struct = struct;
        this.logName = TelemetryConstants.tabPrefix + name;
        checkFMS(false);
        this.datalogPublisher = StructArrayLogEntry.create(DataLogManager.getLog(), logName, struct);
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
            this.ntPublisher = Optional.of(NetworkTableInstance.getDefault().getStructArrayTopic(logName, struct).publish());
        }
    }

    public void update(T[] items) {
        if (items == null) {return;}
        this.datalogPublisher.append(items);
        if (this.ntPublisher.isPresent()) {
            ntPublisher.get().set(items);
            checkFMS(true);
        }
    }
}
