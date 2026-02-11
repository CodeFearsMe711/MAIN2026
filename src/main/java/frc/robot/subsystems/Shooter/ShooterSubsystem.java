package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Shooter subsystem with a software ramp (slew) layer on top of the motor controller's
 * velocity control. Callers set a target RPS (rotations per second) and the subsystem
 * will gradually change the commanded setpoint to that target at a configurable
 * ramp rate (RPS per second). The TalonFX is still used in velocity mode so its
 * internal PID handles closed-loop tracking.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(30);
    private final VelocityVoltage velocityReq = new VelocityVoltage(0);

    // Desired target (what callers want)
    private double desiredRPS = 0.0;
    // Currently commanded setpoint that is actually sent to the motor controller
    private double commandedRPS = 0.0;
    // How fast we allow commandedRPS to change (RPS per second)
    private double rampRPSPerSec = 50.0; // sensible default; tune as needed

    // Time bookkeeping for ramp calculation
    private double lastTime = 0.0;

  public ShooterSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = 0.05;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kV = 0.12;

    motor.getConfigurator().apply(cfg);

    lastTime = Timer.getFPGATimestamp();
  }

  // Low-level: immediately command an RPS to the motor controller (bypasses the software ramp)
  public void setRPS(double rps) {
    // Note: we intentionally send the raw rps through so callers can choose sign.
    // Previously this method inverted the sign here; removing that inversion
    // lets callers control direction explicitly.
    motor.setControl(velocityReq.withVelocity(rps));
    // Keep commandedRPS in sync if someone calls this low-level method
    commandedRPS = rps;
    desiredRPS = rps;
  }

  // Public API: set the target RPS and let the subsystem ramp to it
  public void setTargetRPS(double rps) {
    desiredRPS = rps;
  }

  // Convenience to start spooling to a given RPS
  public void spoolToRPS(double rps) {
    setTargetRPS(rps);
  }

  // Configure the ramp speed (RPS per second)
  public void setRampRPSPerSecond(double rpsPerSec) {
    rampRPSPerSec = Math.max(0.0, rpsPerSec);
  }

  public double getTargetRPS() {
    return desiredRPS;
  }

  public double getCommandedRPS() {
    return commandedRPS;
  }

  // Returns true when the commanded setpoint has reached the desired target within tolerance
  public boolean atTarget(double toleranceRPS) {
    return Math.abs(desiredRPS - commandedRPS) <= Math.abs(toleranceRPS);
  }

  public void stop() {
    setTargetRPS(0);
  }

  @Override
  public void periodic() {
    // Ramp the commanded setpoint toward desiredRPS using rampRPSPerSec
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    if (dt <= 0) {
      dt = 0;
    }
    lastTime = now;

    double maxDelta = rampRPSPerSec * dt;
    double delta = desiredRPS - commandedRPS;

    if (Math.abs(delta) <= maxDelta) {
      commandedRPS = desiredRPS;
    } else {
      commandedRPS += Math.signum(delta) * maxDelta;
    }

    // Send the (possibly ramped) setpoint to the motor controller
    motor.setControl(velocityReq.withVelocity(commandedRPS));
  }
}
