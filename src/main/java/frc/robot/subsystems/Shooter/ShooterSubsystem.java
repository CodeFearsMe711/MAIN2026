package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(30);
    private final VelocityVoltage velocityReq = new VelocityVoltage(0);

  public ShooterSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = 0.12;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kV = 0.12;

    motor.getConfigurator().apply(cfg);
  }

  // Command motor speed in rotations per second
  public void setRPS(double rps) {
    // Note: we intentionally send the raw rps through so callers can choose sign.
    // Previously this method inverted the sign here; removing that inversion
    // lets callers control direction explicitly.
    motor.setControl(velocityReq.withVelocity(rps));
  }

  public void stop() {
    setRPS(0);
  }
}
