package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFeederSubsytem extends SubsystemBase {
    
     private final TalonFX motor = new TalonFX(36);
  private final VelocityVoltage velocityReq = new VelocityVoltage(0);
  private final VoltageOut voltageReq = new VoltageOut(0);

  public ShooterFeederSubsytem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = 0.12;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kV = 0.12;

    motor.getConfigurator().apply(cfg);
  }

  // Command motor speed in rotations per second
  public void setRPS(double rps) {
    motor.setControl(velocityReq.withVelocity(-rps));
  }

  public void setReverseRPS(double rps) {
    setRPS(-Math.abs(rps));
  }

  public void setReverseVoltage(double volts) {
    motor.setControl(voltageReq.withOutput(Math.abs(volts)));
  }

  public void stop() {
    setRPS(0);
  }
}
