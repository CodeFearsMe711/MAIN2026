package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX motor = new TalonFX(30);
  private final VelocityVoltage velocityReq = new VelocityVoltage(0);

  private double desiredRPS = 0.0;
  private double commandedRPS = 0.0;
  private double rampRPSPerSec = 50.0;
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

  public void setRPS(double rps) {
    desiredRPS = rps;
    commandedRPS = rps;
    motor.setControl(velocityReq.withVelocity(rps));
  }

  public void setTargetRPS(double rps) {
    desiredRPS = rps;
  }

  public void spoolToRPS(double rps) {
    setTargetRPS(rps);
  }

  public void setRampRPSPerSecond(double rpsPerSec) {
    rampRPSPerSec = Math.max(0.0, rpsPerSec);
  }

  public double getTargetRPS() {
    return desiredRPS;
  }

  public double getCommandedRPS() {
    return commandedRPS;
  }

  public boolean atTarget(double toleranceRPS) {
    return Math.abs(desiredRPS - commandedRPS) <= Math.abs(toleranceRPS);
  }

  public void stop() {
    desiredRPS = 0.0;
    commandedRPS = 0.0;
    motor.setControl(velocityReq.withVelocity(0.0));
  }

  public void stopAndClearTarget() {
    stop();
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    if (dt < 0.0) {
      dt = 0.0;
    }
    lastTime = now;

    double maxDelta = rampRPSPerSec * dt;
    double delta = desiredRPS - commandedRPS;

    if (Math.abs(delta) <= maxDelta) {
      commandedRPS = desiredRPS;
    } else {
      commandedRPS += Math.signum(delta) * maxDelta;
    }

    motor.setControl(velocityReq.withVelocity(commandedRPS));
  }
}