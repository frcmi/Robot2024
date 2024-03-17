package frc.lib.ultralogger;

import frc.robot.Constants.TelemetryConstants;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.Optional;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class UltraTempLog {
    private double lastUpdated = System.currentTimeMillis();
    private DoubleSupplier tempSupplier;
    private UltraDoubleLog logEntry;
    private static final double TIME_DELAY = TimeUnit.SECONDS.toMillis(2);

    public UltraTempLog(String name, Supplier<Double> tempSupplier) {
        logEntry = new UltraDoubleLog(name);
        this.tempSupplier = tempSupplier::get;
    }
    
    public UltraTempLog(String name, DoubleSupplier tempSupplier) {
        logEntry = new UltraDoubleLog(name);
        this.tempSupplier = tempSupplier;
    }

    public void update() {
        if (System.currentTimeMillis() - lastUpdated < TIME_DELAY) {
            return;
        }
        lastUpdated = System.currentTimeMillis();

        logEntry.update(tempSupplier.getAsDouble());
    }
}
