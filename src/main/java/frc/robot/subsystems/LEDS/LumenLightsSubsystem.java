package frc.robot.subsystems.LEDS;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * No-op LED subsystem stub.
 * Replaces hardware code with safe methods so the robot can run without LED errors.
 * Restore the original implementation only after wiring and driver/library are verified.
 */
public class LumenLightsSubsystem extends SubsystemBase {
    // no hardware here — safe defaults
    public LumenLightsSubsystem() {
        // no-op
    }


    @Override
    public void periodic() {
        // no-op
    }


    /** Safe public API: do nothing. */
    public void setAllRGB(int r, int g, int b) {
        // no-op
    }


    /** Safe public API: attempt to stop (no hardware). */
    public void stop() {
        // no-op
    }


    /** Diagnostics: indicate LEDs are disabled in this build. */
    public boolean isInitialized() {
        return false;
    }


    public String getStatus() {
        return "disabled";
    }


    // keep other helper method names used elsewhere to avoid compile errors
    public int getConfiguredPort() { return -1; }
    public int getConfiguredLength() { return -1; }
    public String getLastError() { return null; }
    public String getLastScanResult() { return null; }
    public int getActivePort() { return -1; }
    public boolean isScanInProgress() { return false; }
    public void reinit() { /* no-op */ }
    public boolean testSetColor(int r, int g, int b) { return false; }
    public void cycleTest(int cycles) { /* no-op */ }
    public void scanPortsAsync(int maxPort) { /* no-op */ }
}
