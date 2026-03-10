package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(30);
  private final VelocityVoltage velocityReq = new VelocityVoltage(0);

  private boolean manualLow = false;
  private boolean manualFast = false;
  private boolean visionEnabled = false;

  private double visionRPS = 0.0;

  public ShooterSubsystem() {

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = 0.05;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kV = 0.12;

    motor.getConfigurator().apply(cfg);
  }

  public void setLowPreset(boolean enabled) {
    manualLow = enabled;
  }

  public void setFastPreset(boolean enabled) {
    manualFast = enabled;
  }

  public void setVisionEnabled(boolean enabled) {
    visionEnabled = enabled;
  }

  public void updateVisionSpeed(double rps) {
    visionRPS = rps;
  }
  public void clearVisionTarget() {
  visionRPS = 0.0;
}

  public void updateVisionTargetRPS(double rps) {
  visionRPS = rps;
}

  private double calculateTarget() {

    if (visionEnabled) {
      return visionRPS;
    }

    if (manualFast) {
      return SmartDashboard.getNumber("Shooter/FastTargetRPS", 125);
    }

    if (manualLow) {
      return SmartDashboard.getNumber("Shooter/TargetRPS", 75);
    }

    return 0;
  }

  public double getMotorRPS() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {

    double target = calculateTarget();

    motor.setControl(velocityReq.withVelocity(target));

    SmartDashboard.putNumber("Shooter/TargetRPS", target);
    SmartDashboard.putNumber("Shooter/MotorRPS", getMotorRPS());
  }
}