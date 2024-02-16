package frc.lib.util;

import java.util.ArrayList;

/**
 * CPU-based synchronization object for waiting on certain events to be
 * triggered. See Vulkan semaphores for more information.
 */
public class Semaphore {
    public Semaphore() {
        status = false;
        dependentCount = 0;
        readyCallbacks = new ArrayList<>();
    }

    /**
     * Essentially pops a dependency if the semaphore has been signaled; otherwise
     * returns false. Make sure whatever calls this is dependent on the semaphore in
     * question.
     * 
     * @return If the semaphore has been signaled.
     */
    public boolean checkStatus() {
        if (status) {
            if (--dependentCount == 0) {
                reset();
            }

            return true;
        }

        return false;
    }

    /**
     * Checks the status of the semaphore. This does not change any state within the system.
     * @return If the semaphore has been signaled.
     */
    public boolean getStatus() {
        return status;
    }

    /**
     * Resets the semaphore state. Does not remove dependents.
     * Calls any callbacks if necessary.
     */
    public void reset() {
        status = false;
        if (dependentCount == 0) {
            for (var callback : readyCallbacks) {
                callback.run();
            }

            readyCallbacks.clear();
        }
    }

    /**
     * Signals the semaphore.
     */
    public void signal() {
        status = true;
    }

    /**
     * Adds a dependent object to the semaphore.
     */
    public void addDependent() {
        dependentCount++;
    }

    /**
     * Checks if the semaphore currently has any dependents.
     */
    public boolean hasDependents() {
        return dependentCount > 0;
    }

    /**
     * Adds a callback to be run when the semaphore is reset.
     * 
     * @param callback
     */
    public void onReady(Runnable callback) {
        if (dependentCount > 0) {
            readyCallbacks.add(callback);
        } else {
            callback.run();
        }
    }

    private boolean status;
    private int dependentCount;
    private ArrayList<Runnable> readyCallbacks;
}
